import QtQuick
import Car_Test2
import QtQuick.Controls

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
                if (stack.currentItem.objectName == "School") {
                    stack.push("Screen04_School_copy.qml");
                } else {
                    stack.push("Screen03_School.qml");
                }



              }
           }






    }



}

