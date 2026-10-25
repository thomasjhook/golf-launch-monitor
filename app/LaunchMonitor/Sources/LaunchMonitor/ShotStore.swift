import Foundation
import GRPCCore
import GRPCNIOTransportHTTP2
import Observation

typealias WireShot = Launchmonitor_V1_ShotResult

enum Connection: Equatable {
    case connecting
    case connected
    case disconnected(String)
}

/// One shot as shown in the UI, converted from SI wire units.
struct Shot: Identifiable, Hashable {
    struct Point: Hashable {
        let downrangeYd: Double
        let heightYd: Double
        let lateralYd: Double
    }

    let id: UUID
    let shotID: UInt32
    let receivedAt: Date
    let ballSpeedMph: Double
    let launchAngleDeg: Double
    let azimuthDeg: Double
    let backspinRpm: Double
    let sidespinRpm: Double
    let carryYd: Double
    let apexFt: Double
    let lateralYd: Double
    let totalYd: Double
    let flightTimeS: Double
    let path: [Point]

    /// Spin is not measured yet, so the service reports zero.
    var hasSpin: Bool { backspinRpm != 0 || sidespinRpm != 0 }

    init(_ wire: WireShot, receivedAt: Date = .now) {
        let yd = 1.0 / 0.9144
        id = UUID()
        shotID = wire.shotID
        self.receivedAt = receivedAt
        ballSpeedMph = wire.launch.ballSpeedMps * 2.236_936
        launchAngleDeg = wire.launch.launchAngleDeg
        azimuthDeg = wire.launch.azimuthDeg
        backspinRpm = wire.launch.backspinRpm
        sidespinRpm = wire.launch.sidespinRpm
        carryYd = wire.flight.carryM * yd
        apexFt = wire.flight.apexHeightM * 3.280_84
        lateralYd = wire.flight.lateralM * yd
        totalYd = wire.flight.totalM * yd
        flightTimeS = wire.flight.flightTimeS
        path = wire.flight.trajectory.map {
            Point(
                downrangeYd: $0.positionM.x * yd,
                heightYd: $0.positionM.y * yd,
                lateralYd: $0.positionM.z * yd)
        }
    }
}

/// Subscribes to `ShotService.StreamShots` and keeps the session's shots.
@MainActor
@Observable
final class ShotStore {
    var shots: [Shot] = []
    var selection: Shot.ID?
    var connection: Connection = .connecting
    var host: String {
        didSet { UserDefaults.standard.set(host, forKey: "serviceHost"); restart() }
    }
    var port: Int {
        didSet { UserDefaults.standard.set(port, forKey: "servicePort"); restart() }
    }

    private var task: Task<Void, Never>?

    init() {
        let env = ProcessInfo.processInfo.environment
        let saved = UserDefaults.standard
        host = env["SHOT_SERVICE_HOST"] ?? saved.string(forKey: "serviceHost") ?? "127.0.0.1"
        let savedPort = saved.integer(forKey: "servicePort")
        port = Int(env["SHOT_SERVICE_PORT"] ?? "") ?? (savedPort != 0 ? savedPort : 8080)
    }

    var selected: Shot? {
        shots.first { $0.id == selection } ?? shots.first
    }

    func start() {
        guard task == nil else { return }
        let host = host, port = port
        // The store lives for the whole app, so the closures capture it strongly.
        task = Task { @MainActor in
            while !Task.isCancelled {
                connection = .connecting
                do {
                    try await Self.stream(
                        host: host, port: port,
                        onHistory: { stored in self.load(history: stored) },
                        onShot: { wire in self.receive(wire) },
                        onOpen: { self.connection = .connected })
                    connection = .disconnected("stream closed")
                } catch is CancellationError {
                    return
                } catch {
                    connection = .disconnected(Self.describe(error))
                }
                try? await Task.sleep(for: .seconds(2))
            }
        }
    }

    func clear() {
        shots.removeAll()
        selection = nil
    }

    private func restart() {
        task?.cancel()
        task = nil
        start()
    }

    /// Replace the list with the service's stored history (newest first).
    private func load(history: [Launchmonitor_V1_StoredShot]) {
        shots = history.map {
            Shot($0.result, receivedAt: Date(timeIntervalSince1970: Double($0.recordedUnixMs) / 1000))
        }
        selection = shots.first?.id
    }

    private func receive(_ wire: WireShot) {
        let shot = Shot(wire)
        shots.insert(shot, at: 0)
        selection = shot.id
    }

    private static func describe(_ error: any Error) -> String {
        if let rpc = error as? RPCError { return rpc.message.isEmpty ? "\(rpc.code)" : rpc.message }
        return String(describing: error)
    }

    private nonisolated static func stream(
        host: String,
        port: Int,
        onHistory: @escaping @MainActor @Sendable ([Launchmonitor_V1_StoredShot]) -> Void,
        onShot: @escaping @MainActor @Sendable (WireShot) -> Void,
        onOpen: @escaping @MainActor @Sendable () -> Void
    ) async throws {
        try await withGRPCClient(
            transport: .http2NIOPosix(
                target: .dns(host: host, port: port),
                transportSecurity: .plaintext)
        ) { client in
            let service = Launchmonitor_V1_ShotService.Client(wrapping: client)
            // History is a nicety: if the database is down, live shots still work.
            do {
                var request = Launchmonitor_V1_ListShotsRequest()
                request.pageSize = 200
                let page = try await service.listShots(request)
                await MainActor.run { onHistory(page.shots) }
            } catch {
                print("could not load shot history: \(error)")
            }
            try await service.streamShots(.init()) { response in
                await MainActor.run { onOpen() }
                for try await shot in response.messages {
                    await MainActor.run { onShot(shot) }
                }
            }
        }
    }
}
