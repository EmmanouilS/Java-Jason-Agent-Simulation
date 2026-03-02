// Agent1.asl — BDI agent for MultiGoalEnv with A* pathing

// -------- Static beliefs --------
move_cost(0.01).
reward_value(green, 0.8).
reward_value(blue, 0.6).
reward_value(yellow, 0.3).
reward_value(purple, 0.2).

// -------- Bootstrap --------
!start.

+!start
  <- .print("Agent1 ready: maximizing net reward with A* pathing.");
     !loop.

// -------- Main loop (wildcard source to avoid Jason errors) --------
+!loop[_] : moves_left(M) & M > 0
  <- !decide_and_step;
     !loop.

+!loop[_] : moves_left(0)
  <- .print("Episode finished; waiting for reset...");
     .wait(200);
     !loop.

// -------- Decision & action based on best_target --------
+!decide_and_step
  : best_target(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NX > X
  <- -current_target(_,_,_);
     +current_target(Color, Steps, Net);
     -current_intention(_);
     +current_intention(pursue(Color));
     .print("Pursuing ", Color, " via right (steps=", Steps, ", net=", Net, ")");
     !go(right).

+!decide_and_step
  : best_target(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NX < X
  <- -current_target(_,_,_);
     +current_target(Color, Steps, Net);
     -current_intention(_);
     +current_intention(pursue(Color));
     .print("Pursuing ", Color, " via left (steps=", Steps, ", net=", Net, ")");
     !go(left).

+!decide_and_step
  : best_target(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NY > Y
  <- -current_target(_,_,_);
     +current_target(Color, Steps, Net);
     -current_intention(_);
     +current_intention(pursue(Color));
     .print("Pursuing ", Color, " via down (steps=", Steps, ", net=", Net, ")");
     !go(down).

+!decide_and_step
  : best_target(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NY < Y
  <- -current_target(_,_,_);
     +current_target(Color, Steps, Net);
     -current_intention(_);
     +current_intention(pursue(Color));
     .print("Pursuing ", Color, " via up (steps=", Steps, ", net=", Net, ")");
     !go(up).

// -------- Fallback to candidate if best_target missing or on spot --------
+!decide_and_step
  : candidate(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NX > X
  <- .print("(Fallback) toward ", Color, " via right");
     !go(right).

+!decide_and_step
  : candidate(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NX < X
  <- .print("(Fallback) toward ", Color, " via left");
     !go(left).

+!decide_and_step
  : candidate(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NY > Y
  <- .print("(Fallback) toward ", Color, " via down");
     !go(down).

+!decide_and_step
  : candidate(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NY < Y
  <- .print("(Fallback) toward ", Color, " via up");
     !go(up).

// Last-resort move if nothing else applies
+!decide_and_step : true <- !go(right).

// -------- Primitive actions --------
+!go(up)    <- move(up).
+!go(down)  <- move(down).
+!go(left)  <- move(left).
+!go(right) <- move(right).

// -------- Reward bookkeeping --------
+received_reward(C,R,At)
  <- .print("Collected ", C, " (", R, ") at t=", At).

+episode_end(TotalR, TotalC, Score, Ep)
  <- .print("EPISODE ", Ep, " END  reward=", TotalR, " cost=", TotalC, " score=", Score).
