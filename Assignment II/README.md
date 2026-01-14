# Adversarial Search – Minimax Algorithm
---

## Project Overview

This project implements an **unbeatable Tic-Tac-Toe AI agent** using the **Minimax Algorithm with Alpha-Beta Pruning**. Designed as a zero-sum, perfect-information game solver, the agent always selects optimal moves to either win or draw, making it impossible to defeat.

**Key Innovation:** The agent employs a **depth-scaled utility function** that prioritizes:
- Winning in the fewest possible moves
- Delaying loss as long as possible when winning is unavoidable
- Adding gameplay variety through random selection when multiple optimal moves exist

---

##  Features

-  Unbeatable Performance: Guaranteed win or draw in every game
-  Optimized Search: Alpha-Beta Pruning eliminates irrelevant branches, improving decision speed
-  Intelligent Heuristics: Uses `10 ' depth` scoring to find fastest victory paths
-  Randomized Optimal Moves: Varied gameplay when multiple equally perfect moves exist
-  Clean Architecture: Object-oriented design separating game engine from AI logic

---

##  Prerequisites

**No external dependencies required!** This project uses only Python standard libraries.

- **Python Version:** 3.6 or higher
- **Required Modules:** `math`, `random`, `time` (all included with Python)

---

##  Installation & Setup

1. **Verify Python Installation:**
   ```bash
   python --version
   ```

2. **Download Project Files:**
   - Download or extract the project folder to your desired directory

3. **File Structure:**
   Ensure `Adversarial Search – Minimax Algorithm.ipynb` is in the project root directory

---

##  How to Play

The game runs entirely in your terminal/console. You play as **'O'** against the AI **'X'**.

### Board Layout
Cell positions are numbered 0 through 8:
```
 0 | 1 | 2
---+---+---
 3 | 4 | 5
---+---+---
 6 | 7 | 8
```

### Gameplay
1. When prompted, enter the number (0-8) of the cell where you want to place your 'O'
2. The AI will immediately respond with its 'X' move
3. The game continues until someone wins or the board fills

---

## Project Structure

```
project-folder/
│
├── Adversarial Search – Minimax Algorithm.ipynb          # Main source code (TicTacToeGame & MinimaxAgent classes)
├── README.md             # This documentation file
└── Design and Implementation of a Minimax.pdf          # Technical report detailing design and algorithms
```

### Code Components
- **`TicTacToeGame`**: Manages game state, board display, and move validation
- **`MinimaxAgent`**: Implements the Minimax algorithm with Alpha-Beta pruning

---

##  Author
**Name:** Fransi Ayele  




