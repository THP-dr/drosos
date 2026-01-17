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
    opacity: 1
    visible: true
    color: "#f4f7f6"
    border.color: "#8b100d0d"




    Rectangle {
        id: rectangle1
        x: 660
        y: 401
        width: 640
        height: 320
        opacity: 0.707
        visible: true

        color: "#ffffff"
        radius: 60
        border.color: "#2c3e50"
        border.width: 6
        clip: false


        Rectangle {
            id: rectangle2
            x: 65
            y: 65
            width: 500
            height: 150
            opacity: 0.5
            color: "#ffffff"
            radius: 20
            border.color: "#2c3e50"
            border.width: 10


        }

        TextInput {
            id: textInput
            x: 195
            y: 279
            width: 250
            height: 20
            text: qsTr("Ξεχάσατε τον κωδικό σας;")
            font.pixelSize: 18
            horizontalAlignment: Text.AlignHCenter
            font.family: "Arial"
            font.underline: true
            cursorVisible: false
            activeFocusOnPress: false
            selectedTextColor: "#2c3e50"
            font.bold: true
            selectionColor: "#2c3e50"
        }
    }



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




    Image {
        id: image
        width: 100
        height: 100
        visible: false
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
        visible: false
        anchors.right: image.left
        anchors.rightMargin: 24

        source: "images/settingsv2.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image2
        x: 1166
        y: 483
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




    Image {
        id: image9
        x: 1672
        y: 24
        width: 100
        height: 100
        anchors.right: image.left
        anchors.rightMargin: 24
        source: "images/eng.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image6
        x: 33
        y: 309
        width: 841
        height: 609
        source: "images/SafeRoute.png"
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
