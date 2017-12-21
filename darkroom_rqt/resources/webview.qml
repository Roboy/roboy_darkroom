import QtQuick 2.2
import QtQuick.Layouts 1.1
import QtQuick.Controls 1.2

ScrollView {
    id: page
    implicitWidth: 640
    implicitHeight: 200

    horizontalScrollBarPolicy: Qt.ScrollBarAlwaysOff

    Item {
        id: content

        width: Math.max(page.viewport.width, grid.implicitWidth + 2 * grid.rowSpacing)
        height: Math.max(page.viewport.height, grid.implicitHeight + 2 * grid.columnSpacing)

        GridLayout {
            id: grid

            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.leftMargin: grid.rowSpacing
            anchors.rightMargin: grid.rowSpacing
            anchors.topMargin: grid.columnSpacing

            columns: page.width < page.height ? 1 : 2

            GroupBox {
                title: "Button"
                Layout.fillWidth: true
                Layout.columnSpan: grid.columns
                RowLayout {
                    anchors.fill: parent
                    Button { text: "OK"; isDefault: true }
                    Button { text: "Cancel" }
                    Item { Layout.fillWidth: true }
                    Button {
                        text: "Attach"
                        menu: Menu {
                            MenuItem { text: "Image" }
                            MenuItem { text: "Document" }
                        }
                    }
                }
            }

            GroupBox {
                title: "CheckBox"
                Layout.fillWidth: true
                ColumnLayout {
                    anchors.fill: parent
                    CheckBox { text: "E-mail"; checked: true }
                    CheckBox { text: "Calendar"; checked: true }
                    CheckBox { text: "Contacts" }
                }
            }

            GroupBox {
                title: "RadioButton"
                Layout.fillWidth: true
                ColumnLayout {
                    anchors.fill: parent
                    ExclusiveGroup { id: radioGroup }
                    RadioButton { text: "Portrait"; exclusiveGroup: radioGroup }
                    RadioButton { text: "Landscape"; exclusiveGroup: radioGroup }
                    RadioButton { text: "Automatic"; exclusiveGroup: radioGroup; checked: true }
                }
            }

            GroupBox {
                title: "Switch"
                Layout.fillWidth: true
                Layout.columnSpan: grid.columns
                ColumnLayout {
                    anchors.fill: parent
                    RowLayout {
                        Label { text: "Wi-Fi"; Layout.fillWidth: true }
                        Switch { checked: true }
                    }
                    RowLayout {
                        Label { text: "Bluetooth"; Layout.fillWidth: true }
                        Switch { checked: false }
                    }
                }
            }
        }
    }
}