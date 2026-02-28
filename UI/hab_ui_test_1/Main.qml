import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQml

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

            background: Rectangle {
                color: "#cfcfcf"
            }

            TabButton {
                text: qsTr("Dashboard")
                background: Rectangle {
                    color: mainTabBar.currentIndex === 0 ? "#ffffff" : '#76cfcfcf'
                    border.color: mainTabBar.currentIndex === 0 ? "#39C5BB" : "#999999"
                    border.width: mainTabBar.currentIndex === 0 ? 2 : 1

                    // Accent underline indicator
                    Rectangle {
                        anchors.bottom: parent.bottom
                        width: parent.width
                        height: 3
                        color: "#39C5BB"
                        visible: mainTabBar.currentIndex === 0
                    }
                }
            }

            TabButton {
                text: qsTr("Console Log")
                background: Rectangle {
                    color: mainTabBar.currentIndex === 1 ? "#ffffff" : "#cfcfcf"
                    border.color: mainTabBar.currentIndex === 1 ? "#39C5BB" : "#999999"
                    border.width: mainTabBar.currentIndex === 1 ? 2 : 1

                    Rectangle {
                        anchors.bottom: parent.bottom
                        width: parent.width
                        height: 3
                        color: "#39C5BB"
                        visible: mainTabBar.currentIndex === 1
                    }
                }
            }

            TabButton {
                text: qsTr("Map")
                background: Rectangle {
                    color: mainTabBar.currentIndex === 2 ? "#ffffff" : "#cfcfcf"
                    border.color: mainTabBar.currentIndex === 2 ? "#39C5BB" : "#999999"
                    border.width: mainTabBar.currentIndex === 2 ? 2 : 1

                    Rectangle {
                        anchors.bottom: parent.bottom
                        width: parent.width
                        height: 3
                        color: "#39C5BB"
                        visible: mainTabBar.currentIndex === 2
                    }
                }
            }
        }

        // 2. Content Area
        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: mainTabBar.currentIndex

            // --- PAGE 1: DASHBOARD ---
            // Currently, every variables are the placeholder.
            ColumnLayout {
                id: dashboardPage
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.margins: 20
                spacing: 20

                // Need to change the variable 
                // --- General Section ---
                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.topMargin: 20
                    Layout.leftMargin: 20
                    Layout.rightMargin: 20
                    spacing: 10

                    Text {
                        text: "⬡  General"
                        color: "#39C5BB"
                        font.pixelSize: 14
                        font.bold: true
                        leftPadding: 4

                        // Optional: accent line next to the title
                        Rectangle {
                            anchors.left: parent.right
                            anchors.leftMargin: 8
                            anchors.verticalCenter: parent.verticalCenter
                            width: 200; height: 1
                            color: "#39C5BB"
                            opacity: 0.3
                        }
                    }

                    // Divider line
                    Rectangle { width: parent.width; height: 1; color: "#333333" }

                    RowLayout {
                        spacing: 20; Layout.fillWidth: true
                        MetricCard { label: "Latitude"; value: currentLat.toFixed(5) + " N"; iconColor: '#39C5BB'; Layout.fillWidth: true }
                        MetricCard { label: "Longitude"; value: currentLat.toFixed(5) + " W"; iconColor: '#39C5BB'; Layout.fillWidth: true }
                        MetricCard { label: "Altitude";    value: currentLat.toFixed(1) + " m";  iconColor: "#39C5BB"; Layout.fillWidth: true }
                        MetricCard { label: "GPS Speed";   value: "12.4 m/s";                    iconColor: "#39C5BB"; Layout.fillWidth: true }
                        MetricCard { label: "Heading";     value: "274 °";                       iconColor: "#39C5BB"; Layout.fillWidth: true }
                        MetricCard { label: "Battery";    value: "3.87 V";             iconColor: "#ffb347"; Layout.fillWidth: true }
                    }
                }

                // --- IMU Section ---
                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.leftMargin: 20
                    Layout.rightMargin: 20
                    spacing: 10

                    Text {
                        text: "⬡  IMU"
                        color: "#bb93ff"
                        font.pixelSize: 14
                        font.bold: true
                        leftPadding: 4

                        Rectangle {
                            anchors.left: parent.right
                            anchors.leftMargin: 8
                            anchors.verticalCenter: parent.verticalCenter
                            width: 200; height: 1
                            color: "#bb93ff"
                            opacity: 0.3
                        }
                    }

                    Rectangle { width: parent.width; height: 1; color: "#333333" }

                    // Acceleration
                    RowLayout {
                        spacing: 20; Layout.fillWidth: true
                        MetricCard { label: "Accel X"; value: "0.12 g";  iconColor: "#bb93ff"; Layout.fillWidth: true }
                        MetricCard { label: "Accel Y"; value: "-0.03 g"; iconColor: "#bb93ff"; Layout.fillWidth: true }
                        MetricCard { label: "Accel Z"; value: "9.81 g";  iconColor: "#bb93ff"; Layout.fillWidth: true }
                    }
                    
                    // Gyro (Not suppose to deploy for the first test)
                    // RowLayout{
                    //     spacing: 20; Layout.fillWidth: true
                    //     MetricCard { label: "Gyro X"; value: "0.12 g";  iconColor: "#bb93ff"; Layout.fillWidth: true }
                    //     MetricCard { label: "Gyro Y"; value: "-0.03 g"; iconColor: "#bb93ff"; Layout.fillWidth: true }
                    //     MetricCard { label: "Gyro Z"; value: "9.81 g";  iconColor: "#bb93ff"; Layout.fillWidth: true }
                    // }
                }

                // --- Radio Section ---
                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.leftMargin: 20
                    Layout.rightMargin: 20
                    spacing: 10

                    Text {
                        text: "⬡  Radio"
                        color: "#ffb347"
                        font.pixelSize: 14
                        font.bold: true
                        leftPadding: 4

                        Rectangle {
                            anchors.left: parent.right
                            anchors.leftMargin: 8
                            anchors.verticalCenter: parent.verticalCenter
                            width: 200; height: 1
                            color: "#ffb347"
                            opacity: 0.3
                        }
                    }

                    Rectangle { width: parent.width; height: 1; color: "#333333" }

                    RowLayout {
                        spacing: 20; Layout.fillWidth: true
                        MetricCard { label: "RSSI";       value: currentRssi + " dBm"; iconColor: currentRssi > -95 ? "#00ff9d" : "#ff4b2b"; Layout.fillWidth: true }
                        MetricCard { label: "Radio Gain"; value: radioGain + " dB";    iconColor: "#ffb347"; Layout.fillWidth: true }
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
        height: 80; color: "#1e1e26"; radius: 10; border.color: "#333333"
        Column {
            anchors.centerIn: parent; spacing: 8
            Text { text: label; color: "#888888"; font.pixelSize: 14; anchors.horizontalCenter: parent.horizontalCenter }
            Text { text: value; color: iconColor; font.pixelSize: 22; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
        }
    }
}
