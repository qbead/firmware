# QBead Circuit

Drag-and-drop single qubit quantum circuit simulator using QBead.

## How To Run:

1. Upload qbeadfirmware/qbeadfirmware.ino to QBead and start it.
2. Open circuit.html
3. Connect to QBead by clicking on Start button (top right) and connecting via Bluetooth.
4. Wait for the connection to succeed. When this happens, "Connection : ON" will be displayed on top of the buttons.
5. Drag-and-drop INIT subroutines or gates to the circuit box.
6. Click on Run button to run your quantum circuit. The corresponding change in state will be displayed in QBead as well.
7. The probability of measuring the qubit in state +Z or -Z after each operation is shown in the graph.


## Gotchas!

1. The Web UI is not responsive, so you might want to run it on a bigger monitor.
2. If no INIT subroutine is added in the beginning, the QBead will be set to state +Z.
3. No way to update the circuit: we have to start from beginning when we click on Reset button.
4. When run on circuit mode, the light in the QBead will not point in the "UP" direction. It will always point to the state of the qubit.
5. If you INIT the qubit with a "minus" state (i.e., -Z, -X, or -Y), due to the un-responsive nature of UI, you might not see the minus (-) sign. However, it will indeed be set in the correct state.

## Required board files

```
https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
```
