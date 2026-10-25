import SwiftUI

struct ContentView: View {
    @Environment(ShotStore.self) private var store

    var body: some View {
        @Bindable var store = store
        NavigationSplitView {
            List(store.shots, selection: $store.selection) { shot in
                ShotRow(shot: shot).tag(shot.id)
            }
            .navigationSplitViewColumnWidth(min: 220, ideal: 260)
            .overlay {
                if store.shots.isEmpty {
                    ContentUnavailableView(
                        "No shots yet", systemImage: "figure.golf",
                        description: Text("Shots appear here as the service processes them."))
                }
            }
        } detail: {
            if let shot = store.selected {
                ShotDetailView(shot: shot)
            } else {
                ConnectionPlaceholder()
            }
        }
        .toolbar {
            ToolbarItem(placement: .navigation) { ConnectionBadge() }
            ToolbarItem {
                Button("Clear", systemImage: "trash") { store.clear() }
                    .disabled(store.shots.isEmpty)
            }
        }
    }
}

private struct ShotRow: View {
    let shot: Shot

    var body: some View {
        HStack {
            VStack(alignment: .leading) {
                Text("Shot \(shot.shotID)").font(.headline)
                Text(shot.receivedAt, style: .time)
                    .font(.caption).foregroundStyle(.secondary)
            }
            Spacer()
            VStack(alignment: .trailing) {
                Text("\(shot.ballSpeedMph, specifier: "%.0f") mph").monospacedDigit()
                Text("\(shot.carryYd, specifier: "%.0f") yd")
                    .font(.caption).foregroundStyle(.secondary).monospacedDigit()
            }
        }
    }
}

private struct ConnectionBadge: View {
    @Environment(ShotStore.self) private var store

    var body: some View {
        HStack(spacing: 6) {
            Circle().fill(color).frame(width: 9, height: 9)
            Text(label).font(.callout).foregroundStyle(.secondary)
        }
        .help("\(store.host):\(store.port)")
    }

    private var color: Color {
        switch store.connection {
        case .connected: .green
        case .connecting: .yellow
        case .disconnected: .red
        }
    }

    private var label: String {
        switch store.connection {
        case .connected: "Connected"
        case .connecting: "Connecting…"
        case .disconnected: "Offline"
        }
    }
}

private struct ConnectionPlaceholder: View {
    @Environment(ShotStore.self) private var store

    var body: some View {
        switch store.connection {
        case .connected:
            ContentUnavailableView(
                "Waiting for a shot", systemImage: "scope",
                description: Text("Connected to \(store.host):\(store.port)."))
        case .connecting:
            ProgressView("Connecting to \(store.host):\(store.port)…")
        case .disconnected(let why):
            ContentUnavailableView(
                "Can't reach shot-service", systemImage: "bolt.horizontal.circle",
                description: Text("\(store.host):\(store.port)\n\(why)\nRetrying every 2 s."))
        }
    }
}
