/*
This is a UI file (.ui.qml) that is intended to be edited in Qt Design Studio only.
It is supposed to be strictly declarative and only uses a subset of QML. If you edit
this file manually, you might introduce QML code that is not supported by Qt Design Studio.
Check out https://doc.qt.io/qtcreator/creator-quick-ui-forms.html for details on .ui.qml files.
*/

import QtQuick
import QtQuick.Controls
import Car_Test2

Rectangle {
    id: rectangle
    width: Constants.width
    height: Constants.height
    color: "#cbcbcb"


    Button {
        id: button
        visible: false
        text: qsTr("Press me")
        anchors.verticalCenter: parent.verticalCenter
        checkable: true
        anchors.horizontalCenter: parent.horizontalCenter

        Connections {
            target: button
            onClicked: animation.start()
        }
    }

    Text {
        id: label
        visible: false
        text: qsTr("Hello Car_Test2")
        anchors.top: button.bottom
        font.family: Constants.font.family
        anchors.topMargin: 45
        anchors.horizontalCenter: parent.horizontalCenter

        SequentialAnimation {
            id: animation

            ColorAnimation {
                id: colorAnimation1
                target: rectangle
                property: "color"
                to: "#2294c6"
                from: Constants.backgroundColor
            }

            ColorAnimation {
                id: colorAnimation2
                target: rectangle
                property: "color"
                to: Constants.backgroundColor
                from: "#2294c6"
            }
        }
    }

    Column {
        id: column
        x: 740
        width: 400
        height: 116
        anchors.verticalCenter: parent.verticalCenter
        spacing: 20

        TextField {
            id: textField
            x: 0
            y: 0
            width: column.width
            text: ""
            horizontalAlignment: Text.AlignLeft
            focus: false
            maximumLength: 24
            cursorVisible: false
            selectionColor: "#eea2a2a2"
            font.pointSize: 25
            font.styleName: "Regular"
            font.family: "Times New Roman"
            placeholderText: qsTr("Username")
        }

        TextField {
            id: textField1
            x: 0
            y: 0
            width: column.width
            text: ""
            horizontalAlignment: Text.AlignLeft
            echoMode: TextInput.PasswordEchoOnEdit
            readOnly: false
            selectionColor: "#eea2a2a2"
            placeholderText: qsTr("Password")
            maximumLength: 16
            font.styleName: "Regular"
            font.pointSize: 25
            font.family: "Times New Roman"
            focus: false
            cursorVisible: false
        }
    }

    Rectangle {
        id: rectangle1
        x: 677
        y: 423
        width: 600
        height: 235
        opacity: 0.2
        color: "#edcb9d"
        radius: 60
    }

    Image {
        id: image
        width: 100
        height: 100
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.rightMargin: 24
        anchors.bottomMargin: 24

        source: "images/light-bulb.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image1
        y: 956
        width: 100
        height: 100
        anchors.right: image.left
        anchors.rightMargin: 24

        source: "images/settingsv2.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image2
        x: 1166
        y: 482
        width: 48
        height: 48
        source: "images/user.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image3
        x: 1166
        y: 550
        width: 48
        height: 48
        source: "images/padlock.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image4
        x: 1086
        y: 550
        width: 48
        height: 48
        source: "images/hide.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image5
        width: 100
        height: 100
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.rightMargin: 24
        anchors.topMargin: 24
        source: "images/brightness-and-contrast.png"
        fillMode: Image.PreserveAspectFit
    }



    states: [
        State {
            name: "clicked"
            when: button.checked

            PropertyChanges {
                target: label
                text: qsTr("Button Checked")
            }
        }
    ]
}
