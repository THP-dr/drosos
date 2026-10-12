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
    color: "#f4f7f6"
    border.color: "#8b100d0d"
    objectName: "School"

    Rectangle {
        id: rectangle1
        x: 14
        y: 82
        width: 600
        height: 955
        color: "#ffffff"
        radius: 60
        border.color: "#2c3e50"
        border.width: 3
        clip: false
    }

    Image {
        id: image
        width: 50
        height: 50
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.rightMargin: 24
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
        id: image5
        width: 50
        height: 50
        anchors.right: parent.right
        anchors.top: parent.top
        anchors.rightMargin: 24
        anchors.topMargin: 24
        source: "images/brightness-and-contrast.png"
        fillMode: Image.PreserveAspectFit
    }

    Column {
        id: column
        width: 504
        height: 712
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.leftMargin: 51
        anchors.bottomMargin: 129
        spacing: 25

        Rectangle {
            id: rectangle2
            x: 0
            y: 8
            width: 510
            height: 129
            color: "#f4f7f6"
            radius: 40
            border.width: 8
            Image {
                id: bus
                x: 8
                y: 35
                width: 128
                height: 106
                source: "images/bus.png"
                fillMode: Image.PreserveAspectFit
            }


        }

        Rectangle {
            id: rectangle3
            x: 0
            y: 8
            width: 504
            height: 129
            color: "#f4f7f6"
            radius: 40
            border.width: 5
            Image {
                id: bus1
                x: 8
                y: 35
                width: 128
                height: 106
                source: "images/bus.png"
                fillMode: Image.PreserveAspectFit
            }

            TextEdit {
                id: textEdit
                x: 142
                y: 38
                width: 330
                height: 54
                text: qsTr("Δρομολόγιο #02")
                font.pixelSize: 44
                font.family: "Arial"
            }
        }

        Rectangle {
            id: rectangle4
            x: 0
            y: 8
            width: 504
            height: 129
            color: "#f4f7f6"
            radius: 40
            border.width: 5
            Image {
                id: bus2
                x: 8
                y: 35
                width: 128
                height: 106
                source: "images/bus.png"
                fillMode: Image.PreserveAspectFit
            }
            TextEdit {
                id: b
                x: 142
                y: 38
                width: 330
                height: 54
                text: qsTr("Δρομολόγιο #03")
                font.pixelSize: 44
                font.family: "Arial"
            }
        }

        Rectangle {
            id: rectangle5
            x: 0
            y: 8
            width: 504
            height: 129
            color: "#f4f7f6"
            radius: 40
            border.width: 5
            Image {
                id: bus3
                x: 8
                y: 35
                width: 128
                height: 106
                source: "images/bus.png"
                fillMode: Image.PreserveAspectFit
            }
            TextEdit {
                id: c
                x: 142
                y: 38
                width: 330
                height: 54
                text: qsTr("Δρομολόγιο #04")
                font.pixelSize: 44
                font.family: "Arial"
            }
        }

        Rectangle {
            id: rectangle6
            x: 0
            y: 8
            width: 504
            height: 129
            color: "#f4f7f6"
            radius: 40
            border.width: 5
            Image {
                id: bus4
                x: 8
                y: 35
                width: 128
                height: 106
                source: "images/bus.png"
                fillMode: Image.PreserveAspectFit
            }
            TextEdit {
                id: d
                x: 142
                y: 38
                width: 330
                height: 54
                text: qsTr("Δρομολόγιο #05")
                font.pixelSize: 44
                font.family: "Arial"
            }
        }




    }



    Image {
        id: image4
        x: 1672
        y: 24
        width: 50
        height: 50
        anchors.right: image.left
        anchors.rightMargin: 24
        source: "images/eng.png"
        fillMode: Image.PreserveAspectFit
    }
    
    
    Text {
        id: text6
        x: 197
        y: 276
        width: 331
        height: 48
        color: "#000000"
        text: "Δρομολόγιο #01"
        font.pixelSize: 44
        horizontalAlignment: Text.AlignLeft
        verticalAlignment: Text.AlignVCenter
        font.family: "Arial"
    }

    Image {
        id: image6
        x: 33
        y: 24
        width: 100
        height: 100
        source: "images/SafeRoute2.png"
        fillMode: Image.PreserveAspectFit

        Rectangle {
            id: rectangle12
            x: 23
            y: 105
            width: 506
            height: 95
            opacity: 0.7
            color: "#ffffff"
            radius: 40
            border.color: "#2c3e50"
            border.width: 5
            clip: false
        }
    }


    states: [
        State {
            name: "clicked"
        }
    ]
    Text {
        id: text5
        x: 358
        y: 153
        width: 170
        height: 50
        color: "#2c3e50"
        text: qsTr("Ιδιότητα")
        font.pixelSize: 44
        verticalAlignment: Text.AlignVCenter
        font.bold: false
        font.family: "Arial"
    }
    Text {
        id: text1
        x: 163
        y: 153
        width: 170
        height: 50
        text: qsTr("Χρήστης")
        font.pixelSize: 44
        verticalAlignment: Text.AlignVCenter
        font.family: "Arial"
    }
    Image {
        id: image2
        x: 94
        y: 155
        width: 50
        height: 50
        source: "images/user.png"
        fillMode: Image.PreserveAspectFit
    }

    Rectangle {
        id: rectangle7
        x: 665
        y: 81
        width: 1231
        height: 900
        color: "#ffffff"
        radius: 60
        border.color: "#2c3e50"
        border.width: 3
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 99

        Text {
            id: text3
            x: 23
            y: 770
            width: 1200
            height: 62
            text: qsTr("Τρέχουσα Θέση: Οδός Π. Ράλλη, κινείται βόρεια. 'Αφιξη σε 25'.")
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 68
            font.pixelSize: 44
            horizontalAlignment: Text.AlignHCenter
            font.family: "Arial"
        }

        Rectangle {
            id: rectangle10
            x: 749
            y: 33
            width: 458
            height: 200
            color: "#f4f7f6"
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
                height: 58
                text: qsTr("Art Garfunkel")
                font.pixelSize: 30
                font.family: "Arial"
            }

            Text {
                id: text9
                x: 103
                y: 129
                width: 229
                height: 83
                text: qsTr("Λεωφορείο #01")
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
        clip: false
    }

    Rectangle {
        id: rectangle11
        x: 1419
        y: 336
        width: 458
        height: 200
        color: "#f4f7f6"
        radius: 40
        border.width: 5
        TextEdit {
            id: textEdit4
            x: 91
            y: 10
            width: 303
            height: 58
            text: qsTr("Λίστα Μαθητών")
            font.pixelSize: 44
            font.family: "Arial"
        }

        TextEdit {
            id: textEdit5
            x: 91
            y: 74
            width: 308
            height: 58
            text: qsTr("Κορτώ Α.")
            font.pixelSize: 30
            font.family: "Arial"
            Slider {
                id: slider
                x: 245
                y: 9
                value: 0.5
                rotation: 270
            }
        }

        Text {
            id: text10
            x: 92
            y: 126
            width: 274
            height: 56
            text: qsTr("Greenwell G.")
            font.pixelSize: 30
            font.family: "Arial"
        }

        Image {
            id: image3
            x: 17
            y: 20
            width: 60
            height: 50

            source: "images/students.png"
            fillMode: Image.PreserveAspectFit
        }

        Image {
            id: image9
            x: 65
            y: 82
            width: 20
            height: 20
            source: "images/red.png"
            fillMode: Image.PreserveAspectFit
        }

        Image {
            id: image10
            x: 66
            y: 137
            width: 20
            height: 20
            source: "images/green.png"
            fillMode: Image.PreserveAspectFit
        }
    }


    Rectangle {
        id: rectangle13
        x: 1419
        y: 574
        width: 458
        height: 200
        color: "#f4f7f6"
        radius: 40
        border.color: "#d32f2f"
        border.width: 7
        Text {
            id: text200
            x: 48
            y: 74
            width: 362
            height: 53
            text: qsTr("Επίβλεψη")
            font.pixelSize: 44
            horizontalAlignment: Text.AlignHCenter
            font.family: "Arial"
        }

        Image {
            id: image11
            x: 32
            y: 63
            width: 75
            height: 75
            source: "images/school-bus.png"
            fillMode: Image.PreserveAspectFit
        }

        Image {
            id: image12
            x: 348
            y: 63
            width: 75
            height: 75
            source: "images/school-bus.png"
            fillMode: Image.PreserveAspectFit
        }
    }





}
