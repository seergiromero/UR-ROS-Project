#include "final_work/chess_interface.hpp"

ChessInterface::ChessInterface()
: Node("chess_interface")
{
  // Initialize empty board
  initializeEmptyBoard();

  // Create action clients
  move_action_client_ = rclcpp_action::create_client<MovePiece>(this, "move_piece");
  kill_action_client_ = rclcpp_action::create_client<KillPiece>(this, "kill_piece");
  castling_action_client_ = rclcpp_action::create_client<Castling>(this, "castling");

  // Subscribe to piece locations
  ubication_sub_ = create_subscription<PieceUbicationArray>(
    "piece_ubication_array_topic", 10,
    std::bind(&ChessInterface::ubicationCallback, this, std::placeholders::_1)
  );

  // Timer to start game after construction
  start_timer_ = this->create_wall_timer(
    500ms,
    [this]() { this->startGame(); }
  );
}

void ChessInterface::initializeEmptyBoard()
{
  const std::vector<std::string> rows = {"1", "2", "3", "4", "5", "6", "7", "8"};
  const std::vector<std::string> files = {"a", "b", "c", "d", "e", "f", "g", "h"};

  for (const auto & row : rows) {
    for (const auto & file : files) {
      board_[file + row] = "";
    }
  }
}

void ChessInterface::ubicationCallback(const PieceUbicationArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(board_mutex_);
  
  // Clear the board
  for (auto & [square, piece] : board_) {
    piece = "";
  }

  // Update with new positions
  for (size_t i = 0; i < msg->names.size(); i++) {
    std::string piece_name = msg->names[i];
    std::string square = msg->squares[i];
    
    // Convert to lowercase: "A7" -> "a7"
    std::transform(square.begin(), square.end(), square.begin(), 
                   [](unsigned char c){ return std::tolower(c); });
    
    if (board_.find(square) != board_.end()) {
      board_[square] = piece_name;
    }
  }
  
  board_initialized_ = true;
}

bool ChessInterface::canCastle(const std::string& castling_type)
{
  std::lock_guard<std::mutex> lock(board_mutex_);
  
  // Check if king and rook are in their initial positions
  std::string king_square = "e8";
  std::string king_piece = board_[king_square];
  
  if (king_piece != "kingB") {
    return false; // King not in initial position
  }
  
  if (castling_type == "kingside" || castling_type == "short") {
    // Kingside castling (O-O)
    std::string rook_square = "h8";
    std::string rook_piece = board_[rook_square];
    
    if (rook_piece != "rookB2") {
      return false; // Rook not in position
    }
    
    // Check if squares between king and rook are empty
    std::vector<std::string> middle_squares = {"f8", "g8"};
    for (const auto& square : middle_squares) {
      if (!board_[square].empty()) {
        return false; // Path not clear
      }
    }
  } else {
    // Queenside castling (O-O-O)
    std::string rook_square = "a8";
    std::string rook_piece = board_[rook_square];
    
    if (rook_piece != "rookB1") {
      return false; // Rook not in position
    }
    
    // Check if squares between king and rook are empty
    std::vector<std::string> middle_squares = {"b8", "c8", "d8"};
    for (const auto& square : middle_squares) {
      if (!board_[square].empty()) {
        return false; // Path not clear
      }
    }
  }
  
  return true;
}

bool ChessInterface::isCastlingMove(const std::string& input, std::string& castling_type)
{
  if (input == "O-O" || input == "o-o" || input == "0-0") {
    castling_type = "kingside";
    return true;
  } else if (input == "O-O-O" || input == "o-o-o" || input == "0-0-0") {
    castling_type = "queenside";
    return true;
  }
  return false;
}

void ChessInterface::startGame()
{
  if (game_started_ || !board_initialized_) return;
  
  start_timer_->cancel();
  game_started_ = true;
  
  // Create separate thread for game to not block ROS spinning
  std::thread game_thread(&ChessInterface::playGame, this);
  game_thread.detach();
}

void ChessInterface::clearScreen()
{
  #ifdef _WIN32
    system("cls");
  #else
    system("clear");
  #endif
}

void ChessInterface::printHeader()
{
  std::cout << "╔═══════════════════════════════════════════════════════════════════════════════╗\n";
  std::cout << "║                          🤖 ROBOTIC CHESS INTERFACE 🤖                       ║\n";
  std::cout << "╠═══════════════════════════════════════════════════════════════════════════════╣\n";
  std::cout << "║  Move: " << std::setw(3) << move_count_ << "                      ";
  std::cout << "Current Turn: " << (white_turn_ ? "⚪ WHITE (Human)" : "⚫ BLACK (Robot)") << std::setw(10) << " ║\n";
  std::cout << "║  Captures - White: " << std::setw(2) << white_captures_ << "  Black: " << std::setw(2) << black_captures_;
  std::cout << "                                        ║\n";
  std::cout << "╚═══════════════════════════════════════════════════════════════════════════════╝\n\n";
}

void ChessInterface::printBoard()
{
  std::lock_guard<std::mutex> lock(board_mutex_);
  
  std::cout << "    ┌───┬───┬───┬───┬───┬───┬───┬───┐\n";
  
  for (int row = 8; row >= 1; --row) {
    std::cout << "  " << row << " │";
    
    for (char file = 'a'; file <= 'h'; ++file) {
      std::string square = std::string(1, file) + std::to_string(row);
      
      // Highlight last move
      bool is_last_move = (square == current_from_ || square == current_to_);
      
      if (board_[square].empty()) {
        if (is_last_move) {
          std::cout << " \033[43m \033[0m │"; // Yellow background for empty squares in last move
        } else {
          std::cout << "   │";
        }
      } else {
        std::string piece_symbol = getPieceSymbol(board_[square]);
        if (is_last_move) {
          std::cout << "\033[43m " << piece_symbol << " \033[0m│"; // Yellow background for last move
        } else {
          std::cout << " " << piece_symbol << " │";
        }
      }
    }
    
    std::cout << " " << row << "\n";
    
    if (row > 1) {
      std::cout << "    ├───┼───┼───┼───┼───┼───┼───┼───┤\n";
    }
  }
  
  std::cout << "    └───┴───┴───┴───┴───┴───┴───┴───┘\n";
  std::cout << "      a   b   c   d   e   f   g   h  \n\n";
}

std::string ChessInterface::getPieceSymbol(const std::string& piece)
{
  if (piece.empty()) return " ";
  
  std::string piece_char = "?";
  bool is_black = (piece.find('B') != std::string::npos);
  
  if (piece.find("rook") != std::string::npos) {
    piece_char = is_black ? "r" : "R";
  } else if (piece.find("knight") != std::string::npos) {
    piece_char = is_black ? "n" : "N";
  } else if (piece.find("bishop") != std::string::npos) {
    piece_char = is_black ? "b" : "B";
  } else if (piece.find("queen") != std::string::npos) {
    piece_char = is_black ? "q" : "Q";
  } else if (piece.find("king") != std::string::npos) {
    piece_char = is_black ? "k" : "K";
  } else if (piece.find("pawn") != std::string::npos) {
    piece_char = is_black ? "p" : "P";
  }
  
  return piece_char;
}

void ChessInterface::printMoveHistory()
{
  if (move_history_.empty()) return;
  
  std::cout << "┌─────────────────────────────────────┐\n";
  std::cout << "│           MOVE HISTORY              │\n";
  std::cout << "├─────────────────────────────────────┤\n";
  
  int displayed_moves = std::min(5, static_cast<int>(move_history_.size()));
  for (int i = move_history_.size() - displayed_moves; i < static_cast<int>(move_history_.size()); ++i) {
    if (i >= 0) {
      std::cout << "│ " << std::setw(2) << ((i/2) + 1) << ". " << std::setw(28) << move_history_[i] << " │\n";
    }
  }
  std::cout << "└─────────────────────────────────────┘\n\n";
}

void ChessInterface::printStatus(const std::string& message, bool is_error)
{
  std::string color = is_error ? "\033[31m" : "\033[32m"; // Red for error, green for success
  std::string prefix = is_error ? "❌ " : "✅ ";
  
  std::cout << "┌─────────────────────────────────────────────────────────────────────────────┐\n";
  std::cout << "│ STATUS: " << color << prefix << message << "\033[0m";
  
  // Pad with spaces to reach the border
  int padding = 70 - message.length();
  for (int i = 0; i < padding; ++i) std::cout << " ";
  std::cout << "│\n";
  std::cout << "└─────────────────────────────────────────────────────────────────────────────┘\n\n";
}

void ChessInterface::playGame()
{
  RCLCPP_INFO(get_logger(), "Board initialized - Starting chess game");
  
  while (rclcpp::ok()) {
    clearScreen();
    printHeader();
    printBoard();
    printMoveHistory();

    if (white_turn_) {
      std::cout << "🎯 WHITE PLAYER TURN (Human)\n";
      std::cout << "┌─────────────────────────────────────────────────────────────────────────────┐\n";
      std::cout << "│ Please make your move on the physical board and press ENTER when ready...  │\n";
      std::cout << "└─────────────────────────────────────────────────────────────────────────────┘\n";
      
      // Clear input buffer and wait for ENTER
      std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      std::cin.get();
      
      // Change turn after human move
      white_turn_ = false;
      move_count_++;
    } else {
      std::cout << "🤖 BLACK PLAYER TURN (Robot)\n";
      std::cout << "┌─────────────────────────────────────────────────────────────────────────────┐\n";
      std::cout << "│ Enter robot move (format: e7 e5 or O-O/O-O-O for castling): ";
      
      std::string input;
      std::getline(std::cin, input);
      
      // Trim whitespace
      input.erase(input.find_last_not_of(" \n\r\t") + 1);
      input.erase(0, input.find_first_not_of(" \n\r\t"));
      
      std::cout << "└─────────────────────────────────────────────────────────────────────────────┘\n";

      // Check if input is castling notation
      std::string castling_type;
      if (isCastlingMove(input, castling_type)) {
        // Handle castling move
        if (!canCastle(castling_type)) {
          printStatus("Castling is not possible! Check piece positions and path.", true);
          std::this_thread::sleep_for(3s);
          continue;
        }

        RCLCPP_INFO(get_logger(), "Sending CASTLING to robot: %s", castling_type.c_str());
        
        // Store current move state for castling
        current_from_ = "e8";
        current_to_ = (castling_type == "kingside") ? "g8" : "c8";
        current_piece_ = "castling";
        current_target_piece_ = "";
        
        // Reset move state
        {
          std::lock_guard<std::mutex> lock(move_mutex_);
          move_completed_ = false;
          move_success_ = false;
          current_stage_ = "";
        }
        
        // Send castling action
        sendCastlingAction(castling_type);
        
        // Wait for action completion
        waitForActionCompletion("CASTLING");
        
        if (move_success_) {
          printStatus("Castling completed successfully!");
          
          // Update board state for castling
          {
            std::lock_guard<std::mutex> lock(board_mutex_);
            if (castling_type == "kingside") {
              // Kingside castling: King e8->g8, Rook h8->f8
              board_["e8"] = "";
              board_["g8"] = "kingB";
              board_["h8"] = "";
              board_["f8"] = "rookB2";
            } else {
              // Queenside castling: King e8->c8, Rook a8->d8
              board_["e8"] = "";
              board_["c8"] = "kingB";
              board_["a8"] = "";
              board_["d8"] = "rookB1";
            }
          }
          
          // Add to move history
          std::string move_notation = (castling_type == "kingside") ? "O-O" : "O-O-O";
          move_history_.push_back(move_notation);
          
          // Change turn
          white_turn_ = true;
          std::this_thread::sleep_for(2s);
        } else {
          printStatus("Castling failed! Try again.", true);
          std::this_thread::sleep_for(3s);
        }
        continue;
      }

      // Parse regular move (existing code)
      std::istringstream iss(input);
      std::string from, to;
      iss >> from >> to;

      // Convert to lowercase for internal validation
      std::transform(from.begin(), from.end(), from.begin(), ::tolower);
      std::transform(to.begin(), to.end(), to.begin(), ::tolower);

      // Validate format
      if (from.size() != 2 || to.size() != 2 || 
          from[0] < 'a' || from[0] > 'h' || from[1] < '1' || from[1] > '8' ||
          to[0] < 'a' || to[0] > 'h' || to[1] < '1' || to[1] > '8') {
        printStatus("Invalid format! Use letter (a-h) and number (1-8) (e.g., e7 e5) or O-O/O-O-O", true);
        std::this_thread::sleep_for(2s);
        continue;
      }

      // Get pieces from board
      std::string moving_piece, target_piece;
      {
        std::lock_guard<std::mutex> lock(board_mutex_);
        
        // Check if there's a piece at origin
        if (board_.find(from) == board_.end() || board_[from].empty()) {
          printStatus("No piece found at origin square!", true);
          std::this_thread::sleep_for(2s);
          continue;
        }

        moving_piece = board_[from];
        target_piece = board_[to]; // Can be empty
      }

      // Validate that it's a black piece
      bool is_black_piece = (moving_piece.find('B') != std::string::npos);
      if (!is_black_piece) {
        printStatus("That's not a black piece!", true);
        std::this_thread::sleep_for(2s);
        continue;
      }

      // Check if target square has a piece (capture scenario)
      bool is_capture = !target_piece.empty();
      
      if (is_capture) {
        // Validate that target piece is white (enemy)
        bool is_white_target = (target_piece.find('B') == std::string::npos);
        if (!is_white_target) {
          printStatus("Cannot capture your own piece!", true);
          std::this_thread::sleep_for(2s);
          continue;
        }
      }

      RCLCPP_INFO(get_logger(), "Sending %s to robot: %s -> %s", 
                 is_capture ? "CAPTURE" : "MOVE", from.c_str(), to.c_str());
      
      // Convert destination to uppercase for action
      std::string to_upper = to;
      std::transform(to_upper.begin(), to_upper.end(), to_upper.begin(), ::toupper);
      
      // Store current move state
      current_from_ = from;
      current_to_ = to;
      current_piece_ = moving_piece;
      current_target_piece_ = target_piece;
      
      // Reset move state
      {
        std::lock_guard<std::mutex> lock(move_mutex_);
        move_completed_ = false;
        move_success_ = false;
        current_stage_ = "";
      }
      
      // Send appropriate action based on whether it's a capture or regular move
      if (is_capture) {
        sendKillPieceAction(moving_piece, target_piece, to_upper);
        black_captures_++;
      } else {
        sendMovePieceAction(moving_piece, to_upper);
      }
      
      // Wait for action completion with status updates
      waitForActionCompletion(is_capture ? "CAPTURE" : "MOVE");
      
      if (move_success_) {
        printStatus("Robot move completed successfully!");
        
        // Update board state
        {
          std::lock_guard<std::mutex> lock(board_mutex_);
          board_[from] = "";
          board_[to] = moving_piece;
        }
        
        // Add to move history
        std::string move_notation = from + "->" + to;
        if (is_capture) {
          move_notation += " (captures " + target_piece + ")";
        }
        move_history_.push_back(move_notation);
        
        // Change turn only if move was successful
        white_turn_ = true;
        std::this_thread::sleep_for(2s);
      } else {
        printStatus("Robot move failed! Try again.", true);
        std::this_thread::sleep_for(3s);
      }
    }
  }
}

void ChessInterface::waitForActionCompletion(const std::string& action_type)
{
  auto start_time = std::chrono::steady_clock::now();
  
  while (true) {
    {
      std::unique_lock<std::mutex> lock(move_mutex_);
      if (move_cv_.wait_for(lock, 100ms, [this] { return move_completed_; })) {
        break; // Action completed
      }
    }
    
    // Print status update
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    
    std::string status_msg = "🤖 Robot executing " + action_type + "... (" + 
                            std::to_string(seconds) + "s";
    if (!current_stage_.empty()) {
      status_msg += " - " + current_stage_;
    }
    status_msg += ")";
    
    clearScreen();
    printHeader();
    printBoard();
    printStatus(status_msg);
    
    if (!rclcpp::ok()) break;
  }
}

void ChessInterface::sendCastlingAction(const std::string& castling_type)
{
  // Wait for action server
  if (!castling_action_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(get_logger(), "Castling action server not available");
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      move_completed_ = true;
      move_success_ = false;
    }
    move_cv_.notify_one();
    return;
  }

  // Create goal
  auto goal = Castling::Goal();
  goal.castling_type = castling_type;

  // Configure callbacks
  auto send_goal_options = rclcpp_action::Client<Castling>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ChessInterface::castlingGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&ChessInterface::castlingFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ChessInterface::castlingResultCallback, this, std::placeholders::_1);

  // Send goal asynchronously
  castling_action_client_->async_send_goal(goal, send_goal_options);
}

void ChessInterface::sendMovePieceAction(const std::string & piece_name, const std::string & square)
{
  // Wait for action server
  if (!move_action_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(get_logger(), "Move action server not available");
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      move_completed_ = true;
      move_success_ = false;
    }
    move_cv_.notify_one();
    return;
  }

  // Create goal
  auto goal = MovePiece::Goal();
  goal.piece_name = piece_name;
  goal.square = square;

  // Configure callbacks
  auto send_goal_options = rclcpp_action::Client<MovePiece>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ChessInterface::moveGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&ChessInterface::moveFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ChessInterface::moveResultCallback, this, std::placeholders::_1);

  // Send goal asynchronously
  move_action_client_->async_send_goal(goal, send_goal_options);
}

void ChessInterface::sendKillPieceAction(const std::string & killer_piece, const std::string & killed_piece, const std::string & target_square)
{
  // Wait for action server
  if (!kill_action_client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(get_logger(), "Kill action server not available");
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      move_completed_ = true;
      move_success_ = false;
    }
    move_cv_.notify_one();
    return;
  }

  // Create goal
  auto goal = KillPiece::Goal();
  goal.killer_piece = killer_piece;
  goal.killed_piece = killed_piece;
  goal.target_square = target_square;

  // Configure callbacks
  auto send_goal_options = rclcpp_action::Client<KillPiece>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ChessInterface::killGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&ChessInterface::killFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&ChessInterface::killResultCallback, this, std::placeholders::_1);

  // Send goal asynchronously
  kill_action_client_->async_send_goal(goal, send_goal_options);
}

void ChessInterface::castlingGoalResponseCallback(const GoalHandleCastling::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Castling goal rejected by server");
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      move_completed_ = true;
      move_success_ = false;
    }
    move_cv_.notify_one();
  } else {
    RCLCPP_INFO(get_logger(), "Castling goal accepted, executing...");
  }
}

void ChessInterface::castlingFeedbackCallback(
  GoalHandleCastling::SharedPtr,
  const std::shared_ptr<const Castling::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(move_mutex_);
  current_stage_ = feedback->stage;
  if (!feedback->current_piece.empty()) {
    current_stage_ += " (" + feedback->current_piece + ")";
  }
}

void ChessInterface::castlingResultCallback(const GoalHandleCastling::WrappedResult & result)
{
  bool success = false;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      success = result.result->success;
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Castling action failed");
      success = false;
      break;
  }
  
  {
    std::lock_guard<std::mutex> lock(move_mutex_);
    move_completed_ = true;
    move_success_ = success;
  }
  move_cv_.notify_one();
}

void ChessInterface::moveGoalResponseCallback(const GoalHandleMovePiece::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Move goal rejected by server");
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      move_completed_ = true;
      move_success_ = false;
    }
    move_cv_.notify_one();
  } else {
    RCLCPP_INFO(get_logger(), "Move goal accepted, executing...");
  }
}

void ChessInterface::moveFeedbackCallback(
  GoalHandleMovePiece::SharedPtr,
  const std::shared_ptr<const MovePiece::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(move_mutex_);
  current_stage_ = feedback->stage;
}

void ChessInterface::moveResultCallback(const GoalHandleMovePiece::WrappedResult & result)
{
  bool success = false;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      success = result.result->success;
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Move action failed");
      success = false;
      break;
  }
  
  {
    std::lock_guard<std::mutex> lock(move_mutex_);
    move_completed_ = true;
    move_success_ = success;
  }
  move_cv_.notify_one();
}

void ChessInterface::killGoalResponseCallback(const GoalHandleKillPiece::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Kill goal rejected by server");
    {
      std::lock_guard<std::mutex> lock(move_mutex_);
      move_completed_ = true;
      move_success_ = false;
    }
    move_cv_.notify_one();
  } else {
    RCLCPP_INFO(get_logger(), "Kill goal accepted, executing...");
  }
}

void ChessInterface::killFeedbackCallback(
  GoalHandleKillPiece::SharedPtr,
  const std::shared_ptr<const KillPiece::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(move_mutex_);
  current_stage_ = feedback->stage + " (" + std::to_string(feedback->progress) + "%)";
}

void ChessInterface::killResultCallback(const GoalHandleKillPiece::WrappedResult & result)
{
  bool success = false;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      success = result.result->success;
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Kill action failed");
      success = false;
      break;
  }
  
  {
    std::lock_guard<std::mutex> lock(move_mutex_);
    move_completed_ = true;
    move_success_ = success;
  }
  move_cv_.notify_one();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChessInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}