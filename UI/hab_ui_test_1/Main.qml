import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: root
    visible: true
    width: 1000
    height: 750
    title: "HAB Ground Station Test Version"
    color: "#121212"

    // --- Shared Data & Logic ---
    // Currently, everything is placeholder here.
    property bool followNewData: true
    property string timeFilter: ""
    property double currentLat: 34.0522
    property int currentRssi: -75
    property int radioGain: 20

    ListModel { id: telemetryModel }

    Timer {
        interval: 2000; running: true; repeat: true
        onTriggered: {
            let timestamp = Qt.formatDateTime(new Date(), "yyyy/MM/dd hh:mm:ss")
            telemetryModel.append({
                "timestamp": timestamp,
                "rawContent": "7E 00 12 " + Math.floor(Math.random()*255).toString(16).toUpperCase()
            })
            currentLat += (Math.random() - 0.5) * 0.0005
            currentRssi = -60 - Math.floor(Math.random() * 40)
        }
    }

    // --- Main Layout ---
    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // 1. Navigation Tab Bar
        TabBar {
            id: mainTabBar
            Layout.fillWidth: true
            z: 2 // Keep it on top of content to ensure clicks work

            TabButton { text: "DASHBOARD" }
            Rectangle { width: 2; height: mainTabBar.height * 0.6; color: "#444444" }

            TabButton { text: "RAW DATA CONSOLE" }
            Rectangle { width: 2; height: mainTabBar.height * 0.6; color: "#444444" }

            TabButton { text: "MAP" }
            Rectangle { width: 2; height: mainTabBar.height * 0.6; color: "#444444" }

            background: Rectangle { color: "#1e1e26" }
        }

        // 2. Content Area
        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: mainTabBar.currentIndex

            // --- PAGE 1: DASHBOARD ---
            ColumnLayout {
                id: dashboardPage
                // FIX 2: Remove anchors.fill! Use Layout properties instead.
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.margins: 20
                spacing: 20

                RowLayout {
                    Layout.topMargin: 30
                    Layout.leftMargin: 20
                    Layout.rightMargin: 20
                    spacing: 20
                    MetricCard {
                        label: "COORDINATES"
                        value: currentLat.toFixed(5) + " N"
                        iconColor: "#fab387"; Layout.fillWidth: true
                    }
                    MetricCard {
                        label: "RADIO GAIN"
                        value: currentRssi + " dBm"
                        iconColor: currentRssi > -95 ? "#00ff9d" : "#ff4b2b"; Layout.fillWidth: true
                    }
                }

                Item { Layout.fillHeight: true }
            }

            // --- PAGE 2: CONSOLE LOG ---
            ColumnLayout {
                id: consolePage
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.margins: 20

                RowLayout {
                    TextField {
                        placeholderText: "Filter by time..."
                        Layout.fillWidth: true
                        color: "white"
                        onTextChanged: root.timeFilter = text
                        background: Rectangle { color: "#313244"; radius: 4 }
                    }
                    Button {
                        text: root.followNewData ? "AUTO-SCROLL: ON" : "AUTO-SCROLL: OFF"
                        onClicked: root.followNewData = !root.followNewData
                    }
                }

                Rectangle {
                    Layout.fillWidth: true; Layout.fillHeight: true
                    color: "#1e1e26"; radius: 8; clip: true

                    ListView {
                        id: logView
                        anchors.fill: parent; anchors.margins: 5
                        model: telemetryModel
                        delegate: logDelegate
                        ScrollBar.vertical: ScrollBar { active: true; policy: ScrollBar.AlwaysOn }
                        onCountChanged: if (root.followNewData) logView.positionViewAtEnd()
                    }
                }
            }

            // --- PAGE 3: MAP (Placeholder) ---
            Rectangle {
                color: "#121212"
                Text {
                    anchors.centerIn: parent
                    text: "Map View Coming Soon..."
                    color: "gray"
                    font.pixelSize: 24
                }
            }
        }
    }

    // --- Reusable Components ---
    Component {
        id: logDelegate
        Rectangle {
            width: logView.width; height: isVisible ? 35 : 0; visible: isVisible
            color: index % 2 === 0 ? "transparent" : "#252530"
            readonly property bool isVisible: timestamp.includes(root.timeFilter)
            RowLayout {
                anchors.fill: parent; anchors.leftMargin: 15
                Text { text: "[" + timestamp + "]"; color: "#666666"; font.family: "Monospace" }
                Text {
                    text: rawContent; color: "#00ff9d"; font.family: "Monospace"
                    Layout.fillWidth: true; horizontalAlignment: Text.AlignHCenter
                }
            }
        }
    }

    component MetricCard : Rectangle {
        property string label: ""; property string value: ""; property color iconColor: "white"
        height: 120; color: "#1e1e26"; radius: 10; border.color: "#333333"
        Column {
            anchors.centerIn: parent; spacing: 8
            Text { text: label; color: "#888888"; font.pixelSize: 12; anchors.horizontalCenter: parent.horizontalCenter }
            Text { text: value; color: iconColor; font.pixelSize: 28; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
        }
    }
}
