import Charts
import SwiftUI

struct ShotDetailView: View {
    let shot: Shot

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                header
                metrics
                HStack(alignment: .top, spacing: 16) {
                    chartCard("Side view", yTitle: "Height (yd)") { SideChart(shot: shot) }
                    chartCard("Top view", yTitle: "Downrange (yd)") { TopChart(shot: shot) }
                }
                if !shot.hasSpin {
                    Label(
                        "Spin isn't measured yet, so carry, apex and total assume a zero-spin ball. Trust speed and angles.",
                        systemImage: "info.circle"
                    )
                    .font(.callout).foregroundStyle(.secondary)
                }
            }
            .padding(24)
        }
        .navigationTitle("Shot \(shot.shotID)")
    }

    private var header: some View {
        VStack(alignment: .leading, spacing: 2) {
            Text("\(shot.carryYd, specifier: "%.0f")")
                .font(.system(size: 72, weight: .semibold, design: .rounded))
                .monospacedDigit()
            + Text(" yd carry").font(.title2).foregroundColor(.secondary)
            Text("\(shot.totalYd, specifier: "%.0f") yd total (approx.) · \(directionLabel(shot.lateralYd, "yd")) · \(shot.flightTimeS, specifier: "%.1f") s flight")
                .foregroundStyle(.secondary)
        }
    }

    private var metrics: some View {
        LazyVGrid(columns: [GridItem(.adaptive(minimum: 150), spacing: 12)], spacing: 12) {
            MetricTile("Ball speed", String(format: "%.1f", shot.ballSpeedMph), "mph")
            MetricTile("Launch angle", String(format: "%.1f", shot.launchAngleDeg), "°")
            MetricTile("Launch direction", directionLabel(shot.azimuthDeg, "°"), nil)
            MetricTile("Apex", String(format: "%.0f", shot.apexFt), "ft")
            MetricTile("Backspin", shot.hasSpin ? String(format: "%.0f", shot.backspinRpm) : "—", shot.hasSpin ? "rpm" : nil)
            MetricTile("Sidespin", shot.hasSpin ? String(format: "%+.0f", shot.sidespinRpm) : "—", shot.hasSpin ? "rpm" : nil)
        }
    }

    private func chartCard<C: View>(_ title: String, yTitle: String, @ViewBuilder _ chart: () -> C) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            Text(title).font(.headline)
            chart().frame(height: 260)
        }
        .padding(14)
        .background(.quaternary.opacity(0.4), in: RoundedRectangle(cornerRadius: 12))
    }

    private func directionLabel(_ value: Double, _ unit: String) -> String {
        let magnitude = String(format: unit == "°" ? "%.1f" : "%.1f", abs(value))
        if abs(value) < 0.05 { return "Straight" }
        return "\(magnitude)\(unit == "°" ? "°" : " \(unit)") \(value > 0 ? "R" : "L")"
    }
}

private struct MetricTile: View {
    let title: String
    let value: String
    let unit: String?

    init(_ title: String, _ value: String, _ unit: String?) {
        self.title = title
        self.value = value
        self.unit = unit
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 4) {
            Text(title).font(.caption).foregroundStyle(.secondary)
            HStack(alignment: .firstTextBaseline, spacing: 3) {
                Text(value).font(.system(.title, design: .rounded, weight: .semibold)).monospacedDigit()
                if let unit { Text(unit).foregroundStyle(.secondary) }
            }
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(14)
        .background(.quaternary.opacity(0.4), in: RoundedRectangle(cornerRadius: 12))
    }
}

private struct SideChart: View {
    let shot: Shot

    var body: some View {
        Chart(Array(shot.path.enumerated()), id: \.offset) { _, p in
            LineMark(x: .value("Downrange (yd)", p.downrangeYd), y: .value("Height (yd)", p.heightYd))
                .foregroundStyle(.tint)
        }
        .chartYScale(domain: 0...max(shot.apexFt / 3 * 1.2, 10))
        .chartXScale(domain: 0...max(shot.carryYd * 1.1, 50))
    }
}

private struct TopChart: View {
    let shot: Shot

    var body: some View {
        let half = max(abs(shot.lateralYd) * 1.5, 20)
        Chart(Array(shot.path.enumerated()), id: \.offset) { _, p in
            LineMark(x: .value("Lateral (yd)", p.lateralYd), y: .value("Downrange (yd)", p.downrangeYd))
                .foregroundStyle(.tint)
        }
        .chartXScale(domain: -half...half)
        .chartYScale(domain: 0...max(shot.carryYd * 1.1, 50))
    }
}
