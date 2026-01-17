import QtQuick
import Car_Test2
import QtQuick.Controls
import QtWebSockets
import QtLocation
import QtPositioning
import QtMultimedia

Window {
    width: 1920
    height: 1080

    visible: true
    title: "Car_Test2"

    property string userName: ""

    StackView {
           id: stack
           anchors.fill: parent


           initialItem: Item {
               id: loginRoot

               // Your background screen
               Screen01 {
                   id: mainScreen2
                   anchors.fill: parent


               }

               Column {
                   id: column
                   x: 740
                   width: 400
                   height: 116
                   anchors.verticalCenter: parent.verticalCenter
                   spacing: 20

                   visible: stack.depth === 1

                   TextField {
                       id: textField
                       x: 0
                       y: 0
                       width: column.width
                       opacity: 1
                       horizontalAlignment: Text.AlignLeft
                       placeholderTextColor: "#000000"
                       focus: false
                       maximumLength: 24
                       cursorVisible: false
                       selectionColor: "#2c3e50"
                       font.pointSize: 25
                       font.styleName: "Regular"
                       font.family: "Arial"
                       placeholderText: qsTr("'Ονομα χρήστη")
                       // Update the variable every time the user types
                        onTextChanged: userName = text
                        onAccepted: {
                            if (userName === "school") {
                                console.log("Welcome, School User!")
                                stack.push("Screen03_School.qml")
                                // Trigger school-specific logic here
                            } else if (userName === "parent") {
                                console.log("Welcome, Parent User!")
                                stack.push("Screen02_Parents.qml")
                                // Trigger parent-specific logic here
                            } else {
                                console.log("Unknown user type: " + userName)
                            }
                        }

                   }

                   TextField {
                       id: textField1
                       x: 0
                       y: 0
                       width: column.width
                       text: ""
                       horizontalAlignment: Text.AlignLeft
                       placeholderTextColor: "#000000"
                       echoMode: TextInput.PasswordEchoOnEdit
                       readOnly: false
                       selectionColor: "#2c3e50"
                       placeholderText: qsTr("Κωδικός")
                       maximumLength: 16
                       font.styleName: "Regular"
                       font.pointSize: 25
                       font.family: "Arial"
                       focus: false
                       cursorVisible: false
                   }
               }

               Button {
                   id: button1
                   x: 780
                   y: 627
                   width: 400
                   height: 40
                   opacity: 1
                   text: qsTr("Σύνδεση")
                   highlighted: true
                   visible: stack.depth === 1

                   focus: false
                   icon.color: "#ffc107"
                   flat: false
                   font.bold: false
                   font.pointSize: 16
                   font.family: "Arial"
                   MouseArea {
                       id: mouseArea
                       anchors.fill: parent
                       onClicked: {
                               // Accessing the variable directly
                               if (userName === "school") {
                                   console.log("Welcome, School User!")
                                   stack.push("Screen03_School.qml")
                                   // Trigger school-specific logic here
                               } else if (userName === "parent") {
                                   console.log("Welcome, Parent User!")
                                   stack.push("Screen02_Parents.qml")
                                   // Trigger parent-specific logic here
                               } else {
                                   console.log("Unknown user type: " + userName)
                               }
                           }

                   }
                }

           }

           MouseArea {
              id: mouseAreaDash
              x: 1419
              y: 574
              width: 458
              height: 200
              visible: stack.depth >= 2
              z:5
              onClicked: {
                if (stack.currentItem.objectName === "School") {
                    stack.push("Screen04_School_copy.qml");
                } else {
                    stack.push("Screen03_School.qml");
                }



              }
           }
           onCurrentItemChanged: {
               // If we just landed on the Parents screen, clear old ones immediately
               if (currentItem && currentItem.objectName === "Parents") {
                   while (notificationModel.count > 3) {
                       notificationModel.remove(notificationModel.count - 1);
                   }
               }
           }
    }

    WebSocket {
        id: espSocket
        url: "ws://esp32s3-data.local:81"
        active: true
        property bool smokeDetected: false
        property bool flameDetected: false
        property bool vibrationDetected: false
        property bool arrivalDetected: false
        property bool departureDetected: false
        property bool busDelayDetected: false
        property bool speedLimitDetected: false

        onTextMessageReceived: function(message) {
            let data = JSON.parse(message);

            // Update the Map's center coordinate
            // Assuming your Map has id: map
            map.center = QtPositioning.coordinate(data.lat, data.lng);
            espMarker.coordinate = QtPositioning.coordinate(data.lat, data.lng);

            // Only trigger if data.smoke is true AND it wasn't true before
            if (data.smoke === true && !smokeDetected) {
                addNotification("Καπνός!", "images/danger.png", "#d32f2f");
                smokeDetected = true; // Set state so it doesn't trigger again immediately
            }
            // Reset the state if smoke is cleared, so it can trigger again next time
            else if (data.smoke === false) {
                smokeDetected = false;
            }

            if (data.flame === true && !flameDetected) {
                addNotification("Φωτιά!", "images/danger.png", "#d32f2f");
                flameDetected = true;
            } else if (data.flame === false) {
                flameDetected = false;
            }
            if (data.vibration === true && !vibrationDetected) {
                addNotification("Κραδασμοί!", "images/caution.png", "#fbc02d");
                vibrationDetected = true;
            } else if (data.vibration === false) {
                vibrationDetected = false;
            }
            if (data.arrival === true && !arrivalDetected) {
                addNotification("Αποβίβαση!", "images/tick.png", "#388e3c");
                arrivalDetected = true;
            } else if (data.arrival === false) {
                arrivalDetected = false;
            }
            if (data.departure === true && !departureDetected) {
                addNotification("Επιβίβαση!", "images/tick.png", "#388e3c");
                departureDetected = true;
            } else if (data.departure === false) {
                departureDetected = false;
            }
            if (data.busDelay === true && !busDelayDetected) {
                addNotification("Καθυστερήσεις", "images/caution.png", "#fbc02d");
                busDelayDetected = true;
            } else if (data.busDelay === false) {
                busDelayDetected = false;
            }
            if (data.speedLimit === true && !speedLimitDetected) {
                addNotification("Όριο Ταχύτητας!", "images/caution.png", "#fbc02d");
                speedLimitDetected = true;
            } else if (data.speedLimit === false) {
                speedLimitDetected = false;
            }
        }

        onStatusChanged: {
            if (espSocket.status === WebSocket.Open) {
                console.log("Connected to ESP32!");
                reconnectTimer.stop(); // Stop trying once we are connected
            }
            else if (espSocket.status === WebSocket.Closed || espSocket.status === WebSocket.Error) {
                console.log("Connection lost or failed. Retrying...");
                reconnectTimer.start(); // Start repeating timer
            }
        }
    }

    Timer {
        id: reconnectTimer
        interval: 3000 // Try every 3 seconds
        repeat: true   // KEEP TRYING until stopped
        onTriggered: {
            console.log("Attempting to reconnect...");
            // Toggle active off then on to force a fresh connection attempt
            espSocket.active = false;
            espSocket.active = true;
        }
    }

    Map {
        id: map
        x: 690
        y: 120
        width: 704
        height: 703

        visible: stack.currentItem.objectName === "School" || stack.currentItem.objectName === "Parents"

        plugin: Plugin {
            name: "osm"
            PluginParameter { name: "osm.mapping.custom.host"; value: "https://tile.openstreetmap.org/" }
            PluginParameter { name: "osm.useragent"; value: "MyProjectName/1.0" }
        }

        zoomLevel: 14

        Component.onCompleted: {
            for (var i = 0; i < supportedMapTypes.length; i++) {
                if (supportedMapTypes[i].style === MapType.CustomMap) {
                    activeMapType = supportedMapTypes[i];
                }
            }
        }

        MapQuickItem {
               id: espMarker

               // This ensures the center of your dot is on the exact coordinate
               anchorPoint.x: sourceItem.width / 1.5
               anchorPoint.y: sourceItem.height / 1.5

               sourceItem: Rectangle {
                   width: 20
                   height: 20
                   color: "red"
                   border.color: "white"
                   border.width: 2
                   radius: 10 // Makes it a circle
               }
        }

        MouseArea {
           id: zoomMouseArea
           x: 620
           y: 22
           width: 60
           height: 60
           onClicked: {
               // Increment zoom by 1, staying within the map's allowed limits
               if (map.zoomLevel < map.maximumZoomLevel) {
                   map.zoomLevel += 1;
               }
            }

           Image {
               id: zoomImage
               z:2
               anchors.fill: parent
               source: "images/zoom.png"
               fillMode: Image.PreserveAspectFit
           }

        }

        MouseArea {
           id: unzoomMouseArea
           y: 22
           width: 60
           height: 60
           anchors.right: zoomMouseArea.right
           anchors.rightMargin: 80
           onClicked: {
               // Increment zoom by 1, staying within the map's allowed limits
               if (map.zoomLevel > map.minimumZoomLevel) {
                   map.zoomLevel -= 1;
               }
            }

           Image {
               id: unzoomImage
               z:2
               anchors.fill: parent
               source: "images/unzoom.png"
               fillMode: Image.PreserveAspectFit
           }

        }
    }

    MouseArea {
       id: mouseAreaBack
       x: 30
       y: 22
       width: 60
       height: 60
       visible: stack.depth >= 2
       onClicked: {
             stack.pop(null);
        }

       Image {
           id: backButton
           z:2
           anchors.fill: parent
           source: "images/arrow.png"
           fillMode: Image.PreserveAspectFit
       }

    }

    ListModel {
        id: notificationModel

    }

    function addNotification(message, iconPath, strokeColor) {
        // 1. Insert at index 0 to put it at the top of the Column
        notificationModel.insert(0, {
            msg: message,
            icon: iconPath,
            stroke: strokeColor
        });

        // 2. If we have more than 3, remove the last one (the oldest)
        if (notificationModel.count > 3 && stack.currentItem.objectName === "Parents") {
            notificationModel.remove(notificationModel.count - 1);
        } else if (notificationModel.count > 5) {
            notificationModel.remove(notificationModel.count - 1);
        }
    }

    Column {
        id: columnNotifications
        y: 239
        width: 504
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.leftMargin: 51
        anchors.topMargin: 240
        spacing: 25
        visible: stack.currentItem.objectName === "Parents" || stack.currentItem.objectName === "SchoolCopy"
        Repeater {
            model: notificationModel

            delegate: Rectangle {
                width: 504
                height: 129
                color: "#f4f7f6"
                radius: 40
                border.color: model.stroke // Dynamic color
                border.width: 5

                Image {
                    x: 20
                    anchors.verticalCenter: parent.verticalCenter
                    width: 116
                    height: 92
                    source: model.icon // Dynamic image
                    fillMode: Image.PreserveAspectFit
                }

                Text {
                    x: 170
                    anchors.verticalCenter: parent.verticalCenter
                    color: "#000000"
                    text: model.msg // Dynamic text
                    font.pixelSize: 44
                    font.family: "Arial"
                }
            }
        }

        move: Transition {
            NumberAnimation { properties: "y"; duration: 300; easing.type: Easing.Bezier }
        }

        add: Transition {
            NumberAnimation { property: "opacity"; from: 0; to: 1; duration: 300 }
            NumberAnimation { property: "scale"; from: 0.8; to: 1; duration: 300 }
        }
    }


    Rectangle {
        id: stream
        x: 690
        y: 120
        width: 704
        height: 703
        color: "black"
        visible: stack.currentItem.objectName === "SchoolCopy"

        AnimatedImage {
            id: esp32LiveFeed
            anchors.fill: parent
            source: "http://esp32s3-data.local/stream"
            fillMode: Image.PreserveAspectCrop
            cache: false
            onStatusChanged: {
                if (status === AnimatedImage.Error) {
                    console.log("Stream Error: " + errorString);
                    reconnectionTimer.start();
                }
            }
        }

        Timer {
            id: reconnectionTimer
            interval: 3000
            running: false
            repeat: true
            onTriggered: {
                esp32LiveFeed.source = "";
                esp32LiveFeed.source = "http://esp32s3-data.local/stream";
            }
        }

        Image {
            id: speakerIcon
            x: 8
            y: 17
            width: 100
            height: 100
            source: "images/speaker.png"
        }
    }



 //ALTERNATIVE APPROACH WITH MEDIA PLAYER
   /* Rectangle {
        id: stream
        x: 690
        y: 120
        width: 704
        height: 703
        visible: true
        color: "black"

        MediaPlayer {
            id: mediaplayer
            // Use the mDNS name or the static IP of your ESP32
            source: "http://esp32s3-data.local/stream"
            videoOutput: videoSink

            onErrorOccurred: (error, errorString) => {
                console.log("Stream Error: " + errorString)
                retryTimer.start()
            }
        }

        VideoOutput {
            id: videoSink
            anchors.fill: parent
            fillMode: VideoOutput.PreserveAspectCrop
        }

        Component.onCompleted: mediaplayer.play()

        Timer {
            id: retryTimer
            interval: 3000
            onTriggered: mediaplayer.play()
        }

        Image {
            id: image20
            x: 8
            y: 17
            width: 100
            height: 100
            source: "images/speaker.png"
            fillMode: Image.PreserveAspect
        }
    }*/
 //END OF ALTERNATIVE APPROACH WITH MEDIA PLAYER

}

