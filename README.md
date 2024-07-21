# Hackathon qbead - game

## Bounties

	1. Most creative alternative use of the hardware (see 'Game' section below).
	2. More polished JS library: updated html interface (see 'Interface' section below).

### 1. Game: qubit chaser
This is a self contained game that can be run on the qbead (for installation, just upload `qbeadfirmware/game_qbead.ino` to the qbead).
When the game is uploaded, it can be played as follows.

	1. A white led will be illuminated. This led must be aiming at the ceiling.

	2. *Shake* the qbead to start the game.
![](https://github.com/AlvaroGI/qbead-hackathon/blob/main/gifs/start.gif)

	3. The player is the blue led. A random green qubit will appear. The qbead must be rotated to make the player (blue) *capture the target* (green). Once this happens, a new green qubit will appear at random.
![](https://github.com/AlvaroGI/qbead-hackathon/blob/main/gifs/game.gif)

	4. The goal of the player is to catch as many green qubits as possible before the *timer* (10 seconds) runs out.

	5. After the timer runs out, the qbead goes back to the main menu. It will show the white led again. The *highscore* will be shown as a number of pink leds.
![](https://github.com/AlvaroGI/qbead-hackathon/blob/main/gifs/gameover.gif)

### 2. Interface with JS
We made visual improvements to the html interface. This can be seen in the updated `notifications.html` file, which requires `estilos.css` and `TU-delft.png`.
![](https://github.com/AlvaroGI/qbead-hackathon/blob/main/gifs/interface.gif)
