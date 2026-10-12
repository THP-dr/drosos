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
    color: "#051324"
    border.color: "#8b100d0d"



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

    Rectangle {
        id: rectangle1
        x: 14
        y: 82
        width: 600
        height: 613
        color: "#b0bec5"
        radius: 60
        border.color: "#2c3e50"
        border.width: 3
        clip: false

        Rectangle {
            id: rectangle12
            x: 40
            y: 53
            width: 300
            height: 95
            opacity: 0.7
            color: "#b0bec5"
            radius: 40
            border.color: "#2c3e50"
            border.width: 5
            clip: false
        }
    }

    Image {
        id: image
        x: 1851
        y: 1006
        width: 50
        height: 50
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.rightMargin: 19
        anchors.bottomMargin: 24

        source: "images/light-bulb.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image1
        x: 1772
        y: 1006
        width: 50
        height: 50
        anchors.right: image.left
        anchors.rightMargin: 24

        source: "images/settingsv2.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image2
        x: 80
        y: 153
        width: 48
        height: 48
        source: "images/user.png"
        fillMode: Image.PreserveAspectFit
    }

    Image {
        id: image5
        x: 1851
        width: 50
        height: 50
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.rightMargin: 19
        anchors.topMargin: 26
        source: "images/brightness-and-contrast.png"
        fillMode: Image.PreserveAspectFit
    }

    Column {
        id: column
        y: 239
        width: 504
        height: 439
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.leftMargin: 51
        anchors.bottomMargin: 402
        spacing: 25

        Rectangle {
            id: rectangle2
            x: 0
            y: 8
            width: 504
            height: 129
            color: "#b0bec5"
            radius: 40
            border.color: "#d32f2f"
            border.width: 5
            Image {
                id: bus
                x: 15
                y: 12
                width: 128
                height: 106
                source: "images/danger.png"
                fillMode: Image.PreserveAspectFit
            }


        }

        Rectangle {
            id: rectangle3
            x: 0
            y: 8
            width: 504
            height: 129
            color: "#b0bec5"
            radius: 40
            border.color: "#388e3c"
            border.width: 5
            Image {
                id: bus1
                x: 23
                y: 20
                width: 116
                height: 89
                source: "images/tick.png"
                fillMode: Image.PreserveAspectFit
            }
        }

        Rectangle {
            id: rectangle4
            x: 0
            y: 8
            width: 504
            height: 129
            color: "#b0bec5"
            radius: 40
            border.color: "#fbc02d"
            border.width: 5
            Image {
                id: bus2
                x: 23
                y: 19
                width: 116
                height: 92
                source: "images/caution.png"
                fillMode: Image.PreserveAspectFit
            }
        }
    }

    Text {
        id: text1
        x: 154
        y: 153
        width: 172
        height: 48
        text: qsTr("Χρήστης")
        font.pixelSize: 44
        verticalAlignment: Text.AlignVCenter
        font.family: "Arial"
    }

    Rectangle {
        id: rectangle7
        x: 665
        y: 82
        width: 1236
        height: 900
        color: "#b0bec5"
        radius: 60
        border.color: "#2c3e50"
        border.width: 3
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 98
        clip: false

        AnimatedImage {
            id: animatedImage
            x: 23
            y: 33
            width: 704
            height: 703
            source: "images/map.png"

            Image {
                id: image10
                x: 8
                y: 17
                width: 100
                height: 100
                source: "images/speaker.png"
                fillMode: Image.PreserveAspectFit
            }
        }

        Text {
            id: text3
            x: 23
            y: 782
            width: 1199
            height: 62
            text: qsTr("Τρέχουσα Θέση: Οδός Π. Ράλλη, κινείται βόρεια. 'Αφιξη σε 25'.")
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 56
            font.pixelSize: 44
            horizontalAlignment: Text.AlignLeft
            font.family: "Arial"
        }

        Rectangle {
            id: rectangle8
            x: 751
            y: 41
            width: 458
            height: 200
            color: "#b0bec5"
            radius: 40
            border.width: 5

            TextEdit {
                id: textEdit
                x: 63
                y: 8
                width: 251
                height: 53
                text: qsTr("Κατάσταση:")
                font.pixelSize: 44
                font.family: "Arial"
            }

            TextEdit {
                id: textEdit1
                x: 63
                y: 74
                width: 380
                height: 58
                text: qsTr(" Η Karen επιβιβάστηκε")
                font.pixelSize: 30
                horizontalAlignment: Text.AlignLeft
                font.family: "Arial"
            }

            Text {
                id: text8
                x: 63
                y: 129
                width: 380
                height: 83
                text: qsTr("07:09, Σπίτι")
                font.pixelSize: 30
                font.family: "Arial"
            }

            Image {
                id: image9
                x: 8
                y: 52
                width: 65

                height: 71

                source: "images/check2.png"
                fillMode: Image.PreserveAspectFit
            }
        }

        Rectangle {
            id: rectangle9
            x: 751
            y: 290
            width: 458
            height: 200
            color: "#b0bec5"
            radius: 40
            border.width: 5
            TextEdit {
                id: textEdit2
                x: 103
                y: 10
                width: 213
                height: 58
                text: qsTr("Οδηγός:")
                font.pixelSize: 44
                font.family: "Arial"
            }

            TextEdit {
                id: textEdit3
                x: 103
                y: 65
                width: 189
                height: 41
                text: qsTr("Joan Baez")
                font.pixelSize: 30
                font.family: "Arial"
            }

            Text {
                id: text9
                x: 103
                y: 112
                width: 229
                height: 63
                text: qsTr("Λεωφορείο #09")
                font.pixelSize: 30
                font.family: "Arial"
            }

            Image {
                id: image7
                x: 338
                y: 53
                width: 100
                height: 100
                source: "images/telephone.png"
                fillMode: Image.PreserveAspectFit
            }

            Image {
                id: image8
                x: 21
                y: 10
                width: 76
                height: 71
                source: "images/profile.png"
                fillMode: Image.PreserveAspectFit
            }
        }

        Rectangle {
            id: rectangle10
            x: 744
            y: 536
            width: 458
            height: 200
            color: "#b0bec5"
            radius: 40
            border.color: "#d32f2f"
            border.width: 10


            TextEdit {
                id: textEdit5
                x: 84
                y: 74
                width: 366
                height: 53
                text: qsTr("Δήλωση Απουσίας")
                font.pixelSize: 44
                horizontalAlignment: Text.AlignHCenter
                font.family: "Arial"
            }
        }

        Image {
            id: image6
            x: 758
            y: 603
            width: 69
            height: 69
            source: "images/cross.png"
            fillMode: Image.PreserveAspectFit
        }



    }

    Image {
        id: image4
        x: 1777
        y: 26
        width: 50
        height: 50
        anchors.right: image.left
        anchors.rightMargin: 24
        source: "images/eng.png"
        fillMode: Image.PreserveAspectFit
    }
    
    Text {
        id: text4
        x: 197
        y: 280
        width: 213
        height: 48
        color: "#080808"
        text: "Κίνδυνος"
        font.pixelSize: 44
        horizontalAlignment: Text.AlignLeft
        verticalAlignment: Text.AlignVCenter
        font.family: "Arial"
    }
    
    Text {
        id: text5
        x: 197
        y: 432
        width: 213
        height: 48
        color: "#000000"
        text: "Αποβίβαση"
        font.pixelSize: 44
        horizontalAlignment: Text.AlignLeft
        verticalAlignment: Text.AlignVCenter
        font.family: "Arial"
    }
    
    Text {
        id: text6
        x: 197
        y: 589
        width: 258
        height: 48
        color: "#000000"
        text: "Καθυστέρηση"
        font.pixelSize: 44
        horizontalAlignment: Text.AlignLeft
        verticalAlignment: Text.AlignVCenter
        font.family: "Arial"
    }

    Image {
        id: image3
        x: 0
        y: -30
        width: 235
        height: 293
        visible: false
        source: "images/SafeRouteSmall.png"
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

    Rectangle {
        id: rectangle5
        x: 11
        y: 724
        width: 606
        height: 248
        color: "#b0bec5"
        radius: 60
        border.color: "#2c3e50"
        border.width: 3
        clip: false

        Column {
            id: column2
            x: 44
            y: 20
            width: 510
            height: 209
            anchors.left: parent.left
            anchors.bottom: parent.bottom
            anchors.leftMargin: 48
            anchors.bottomMargin: 20
            spacing: 5
        }
        Rectangle {
            id: rectangle13
            x: 51
            y: 17
            width: 504
            height: 100
            color: "#b0bec5"
            radius: 40
            border.color: "#2c3e50"
            border.width: 8

            Image {
                id: student
                x: 31
                y: 5
                width: 90
                height: 90
                source: "images/student.png"
                fillMode: Image.PreserveAspectFit
            }


            Text {
                id: text2
                x: 118
                y: 21
                text: qsTr("Carpenter K.")
                font.pixelSize: 44
                font.family: "Arial"
            }

            Rectangle {
                id: rectangle14
                x: 0
                y: 115
                width: 504
                height: 100
                color: "#b0bec5"
                radius: 40
                border.color: "#2c3e50"
                border.width: 5

                Image {
                    id: student2
                    x: 31
                    y: 5
                    width: 90
                    height: 90
                    source: "images/student.png"
                    fillMode: Image.PreserveAspectFit
                }
            }



            Text {
                id: text7
                x: 118
                y: 136
                text: qsTr("Carpenter R.")
                font.pixelSize: 44
                font.family: "Arial"
            }



        }
}
}
