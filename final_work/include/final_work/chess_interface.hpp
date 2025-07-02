#ifndef CHESS_INTERFACE_HPP
#define CHESS_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <final_work/action/move_piece.hpp>
#include <final_work/action/kill_piece.hpp>
#include <final_work/action/castling.hpp>
#include <final_work/msg/piece_ubication_array.hpp>
#include <map>
#include <iostream>
#include <string>
#include <vector>
#include <cctype>
#include <algorithm>
#include <mutex>
#include <limits>
#include <future>
#include <condition_variable>
#include <iomanip>
#include <chrono>

using MovePiece = final_work::action::MovePiece;
using KillPiece = final_work::action::KillPiece;
using Castling = final_work::action::Castling;
using GoalHandleMovePiece = rclcpp_action::ClientGoalHandle<MovePiece>;
using GoalHandleKillPiece = rclcpp_action::ClientGoalHandle<KillPiece>;
using GoalHandleCastling = rclcpp_action::ClientGoalHandle<Castling>;
using PieceUbicationArray = final_work::msg::PieceUbicationArray;
using namespace std::chrono_literals;

/**
 * @brief Chess Interface for robotic chess system
 * 
 * This class provides a comprehensive interface for managing a chess game
 * between a human player (white pieces) and a robotic player (black pieces).
 * It handles move validation, board visualization, and communication with
 * robotic chess piece manipulation actions.
 */
class ChessInterface : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for ChessInterface
   * 
   * Initializes the chess interface, creates action clients, sets up
   * subscriptions, and prepares the game environment.
   */
  ChessInterface();

private:
  // ROS2 components
  rclcpp_action::Client<MovePiece>::SharedPtr move_action_client_;       ///< Client for move piece actions
  rclcpp_action::Client<KillPiece>::SharedPtr kill_action_client_;       ///< Client for kill piece actions
  rclcpp_action::Client<Castling>::SharedPtr castling_action_client_;    ///< Client for castling actions
  rclcpp::Subscription<PieceUbicationArray>::SharedPtr ubication_sub_;   ///< Subscription for piece positions
  rclcpp::TimerBase::SharedPtr start_timer_;                             ///< Timer to start the game
  
  // Game state
  std::map<std::string, std::string> board_;                             ///< Chess board representation (square -> piece)
  std::mutex board_mutex_;                                               ///< Mutex for thread-safe board access
  bool white_turn_ = true;                                               ///< Current turn indicator (true = white, false = black)
  bool board_initialized_ = false;                                       ///< Flag indicating if board is initialized
  bool game_started_ = false;                                            ///< Flag indicating if game has started
  int move_count_ = 1;                                                   ///< Current move number
  
  // Move state tracking
  std::string current_from_;                                             ///< Source square of current move
  std::string current_to_;                                               ///< Destination square of current move
  std::string current_piece_;                                            ///< Name of piece being moved
  std::string current_target_piece_;                                     ///< Name of piece being captured (if any)
  
  // Synchronization variables
  std::mutex move_mutex_;                                                ///< Mutex for move synchronization
  std::condition_variable move_cv_;                                      ///< Condition variable for move completion
  bool move_completed_ = false;                                          ///< Flag indicating move completion
  bool move_success_ = false;                                            ///< Flag indicating move success
  std::string current_stage_ = "";                                       ///< Current stage of move execution
  
  // Game statistics
  int white_captures_ = 0;                                               ///< Number of pieces captured by white
  int black_captures_ = 0;                                               ///< Number of pieces captured by black
  std::vector<std::string> move_history_;                                ///< History of all moves made

  /**
   * @brief Initialize an empty chess board
   * 
   * Sets up the internal board representation with all squares
   * mapped to empty strings, ready for piece placement.
   */
  void initializeEmptyBoard();

  /**
   * @brief Callback function for piece location updates
   * 
   * Processes incoming messages about piece positions and updates
   * the internal board representation accordingly.
   * 
   * @param msg Shared pointer to PieceUbicationArray message containing piece positions
   */
  void ubicationCallback(const PieceUbicationArray::SharedPtr msg);

  /**
   * @brief Check if castling move is possible
   * 
   * Validates whether a castling move can be performed by checking
   * piece positions and ensuring the path is clear.
   * 
   * @param castling_type Type of castling ("kingside" or "queenside")
   * @return true if castling is possible, false otherwise
   */
  bool canCastle(const std::string& castling_type);

  /**
   * @brief Check if input string represents a castling move
   * 
   * Parses user input to determine if it represents castling notation
   * and extracts the castling type.
   * 
   * @param input User input string to check
   * @param castling_type Reference to store the detected castling type
   * @return true if input is castling notation, false otherwise
   */
  bool isCastlingMove(const std::string& input, std::string& castling_type);

  /**
   * @brief Start the chess game
   * 
   * Initializes the game loop after ensuring the board is properly
   * initialized and all systems are ready.
   */
  void startGame();

  /**
   * @brief Clear the terminal screen
   * 
   * Provides cross-platform screen clearing functionality for
   * better user interface presentation.
   */
  void clearScreen();

  /**
   * @brief Print the game header with current status
   * 
   * Displays game information including move number, current turn,
   * and capture statistics in a formatted header.
   */
  void printHeader();

  /**
   * @brief Print the current chess board state
   * 
   * Renders the chess board with piece positions, highlighting
   * the last move made for visual feedback.
   */
  void printBoard();

  /**
   * @brief Get the display symbol for a chess piece
   * 
   * Converts internal piece names to single-character symbols
   * for board display purposes.
   * 
   * @param piece Internal piece name string
   * @return Single character symbol representing the piece
   */
  std::string getPieceSymbol(const std::string& piece);

  /**
   * @brief Print the recent move history
   * 
   * Displays the last few moves made in the game for
   * reference and tracking purposes.
   */
  void printMoveHistory();

  /**
   * @brief Print a status message with formatting
   * 
   * Displays status information with appropriate coloring
   * and formatting for success or error messages.
   * 
   * @param message Status message to display
   * @param is_error Flag indicating if this is an error message
   */
  void printStatus(const std::string& message, bool is_error = false);

  /**
   * @brief Main game loop
   * 
   * Manages the turn-based gameplay, handling both human
   * and robot moves, input validation, and game flow.
   */
  void playGame();

  /**
   * @brief Wait for action completion with status updates
   * 
   * Blocks execution while waiting for a robotic action to complete,
   * providing real-time status updates to the user.
   * 
   * @param action_type Type of action being executed ("MOVE", "CAPTURE", "CASTLING")
   */
  void waitForActionCompletion(const std::string& action_type);

  /**
   * @brief Send castling action to the robot
   * 
   * Initiates a castling action by sending the appropriate goal
   * to the castling action server.
   * 
   * @param castling_type Type of castling to perform ("kingside" or "queenside")
   */
  void sendCastlingAction(const std::string& castling_type);

  /**
   * @brief Send move piece action to the robot
   * 
   * Initiates a piece movement action by sending the move goal
   * to the move piece action server.
   * 
   * @param piece_name Name of the piece to move
   * @param square Target square for the piece movement
   */
  void sendMovePieceAction(const std::string & piece_name, const std::string & square);

  /**
   * @brief Send kill piece action to the robot
   * 
   * Initiates a piece capture action by sending the kill goal
   * to the kill piece action server.
   * 
   * @param killer_piece Name of the piece performing the capture
   * @param killed_piece Name of the piece being captured
   * @param target_square Square where the capture occurs
   */
  void sendKillPieceAction(const std::string & killer_piece, const std::string & killed_piece, const std::string & target_square);

  // Castling Action Callbacks
  
  /**
   * @brief Callback for castling goal response
   * 
   * Handles the server's response to a castling goal request,
   * processing acceptance or rejection of the goal.
   * 
   * @param goal_handle Shared pointer to the castling goal handle
   */
  void castlingGoalResponseCallback(const GoalHandleCastling::SharedPtr & goal_handle);

  /**
   * @brief Callback for castling action feedback
   * 
   * Processes feedback messages during castling execution,
   * updating the current stage information.
   * 
   * @param goal_handle Shared pointer to the castling goal handle
   * @param feedback Shared pointer to the castling feedback message
   */
  void castlingFeedbackCallback(
    GoalHandleCastling::SharedPtr goal_handle,
    const std::shared_ptr<const Castling::Feedback> feedback);

  /**
   * @brief Callback for castling action result
   * 
   * Handles the final result of a castling action, determining
   * success or failure and updating the game state accordingly.
   * 
   * @param result Wrapped result containing the castling outcome
   */
  void castlingResultCallback(const GoalHandleCastling::WrappedResult & result);

  // Move Action Callbacks
  
  /**
   * @brief Callback for move goal response
   * 
   * Handles the server's response to a move piece goal request,
   * processing acceptance or rejection of the goal.
   * 
   * @param goal_handle Shared pointer to the move piece goal handle
   */
  void moveGoalResponseCallback(const GoalHandleMovePiece::SharedPtr & goal_handle);

  /**
   * @brief Callback for move action feedback
   * 
   * Processes feedback messages during move execution,
   * updating the current stage information.
   * 
   * @param goal_handle Shared pointer to the move piece goal handle
   * @param feedback Shared pointer to the move piece feedback message
   */
  void moveFeedbackCallback(
    GoalHandleMovePiece::SharedPtr goal_handle,
    const std::shared_ptr<const MovePiece::Feedback> feedback);

  /**
   * @brief Callback for move action result
   * 
   * Handles the final result of a move action, determining
   * success or failure and updating the game state accordingly.
   * 
   * @param result Wrapped result containing the move outcome
   */
  void moveResultCallback(const GoalHandleMovePiece::WrappedResult & result);

  // Kill Action Callbacks
  
  /**
   * @brief Callback for kill goal response
   * 
   * Handles the server's response to a kill piece goal request,
   * processing acceptance or rejection of the goal.
   * 
   * @param goal_handle Shared pointer to the kill piece goal handle
   */
  void killGoalResponseCallback(const GoalHandleKillPiece::SharedPtr & goal_handle);

  /**
   * @brief Callback for kill action feedback
   * 
   * Processes feedback messages during kill execution,
   * updating the current stage and progress information.
   * 
   * @param goal_handle Shared pointer to the kill piece goal handle
   * @param feedback Shared pointer to the kill piece feedback message
   */
  void killFeedbackCallback(
    GoalHandleKillPiece::SharedPtr goal_handle,
    const std::shared_ptr<const KillPiece::Feedback> feedback);

  /**
   * @brief Callback for kill action result
   * 
   * Handles the final result of a kill action, determining
   * success or failure and updating the game state accordingly.
   * 
   * @param result Wrapped result containing the kill outcome
   */
  void killResultCallback(const GoalHandleKillPiece::WrappedResult & result);
};

#endif // CHESS_INTERFACE_HPP