import AppKit
import SwiftUI

@main
struct LaunchMonitorApp: App {
    @State private var store = ShotStore()

    init() {
        // Built with SwiftPM (no .app bundle), so the process starts as a
        // background tool; promote it so it gets a window and a Dock icon.
        NSApplication.shared.setActivationPolicy(.regular)
        NSApplication.shared.activate(ignoringOtherApps: true)
    }

    var body: some Scene {
        WindowGroup("Launch Monitor") {
            ContentView()
                .environment(store)
                .frame(minWidth: 900, minHeight: 600)
                .task { store.start() }
        }
    }
}
