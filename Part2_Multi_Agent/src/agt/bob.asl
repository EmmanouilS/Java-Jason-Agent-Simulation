// BDI agent implementing worker behavior in contract net protocol


// Reference to manager for communication
manager(manager).

// Initialize agent and start main behavior loop
!start.

+!start <-
    .my_name(Me);
    .print(Me, ": ready to participate in auctions.");
    !loop.

// Main behavior loop for agent decision making
+!loop[_] : moves_left(M) & M > 0 & assigned_goal(_) <-
    !decide_and_step;
    !loop.

+!loop[_] : moves_left(M) & M > 0 & not assigned_goal(_) <-
    !idle;
    !loop.

+!loop[_] : moves_left(0) <-
    .print("Episode finished; waiting for reset...");
    .wait(200);
    !loop.

+!loop[_] <-
    .wait(100);
    !loop.

+!idle <-
    .wait(150).

// Decision making hierarchy for goal pursuit

// Use optimal path planning when available
+!decide_and_step
    : assigned_goal(Color) & best_target(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NX > X
<-
    -current_target(_,_,_); +current_target(Color, Steps, Net);
    -current_intention(_);  +current_intention(pursue(Color));
    .print("Pursuing ", Color, " via right (steps=", Steps, ", net=", Net, ")");
    !go(right).

+!decide_and_step
    : assigned_goal(Color) & best_target(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NX < X
<-
    -current_target(_,_,_); +current_target(Color, Steps, Net);
    -current_intention(_);  +current_intention(pursue(Color));
    .print("Pursuing ", Color, " via left (steps=", Steps, ", net=", Net, ")");
    !go(left).

+!decide_and_step
    : assigned_goal(Color) & best_target(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NY > Y
<-
    -current_target(_,_,_); +current_target(Color, Steps, Net);
    -current_intention(_);  +current_intention(pursue(Color));
    .print("Pursuing ", Color, " via down (steps=", Steps, ", net=", Net, ")");
    !go(down).

+!decide_and_step
    : assigned_goal(Color) & best_target(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NY < Y
<-
    -current_target(_,_,_); +current_target(Color, Steps, Net);
    -current_intention(_);  +current_intention(pursue(Color));
    .print("Pursuing ", Color, " via up (steps=", Steps, ", net=", Net, ")");
    !go(up).

// Use candidate information when optimal path not available
+!decide_and_step
    : assigned_goal(Color) & candidate(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NX > X
<-  .print("(Candidate) toward ", Color, " via right");
    !go(right).

+!decide_and_step
    : assigned_goal(Color) & candidate(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NX < X
<-  .print("(Candidate) toward ", Color, " via left");
    !go(left).

+!decide_and_step
    : assigned_goal(Color) & candidate(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NY > Y
<-  .print("(Candidate) toward ", Color, " via down");
    !go(down).

+!decide_and_step
    : assigned_goal(Color) & candidate(Color, NX, NY, Steps, Net) & agent_position(X,Y) & NY < Y
<-  .print("(Candidate) toward ", Color, " via up");
    !go(up).

// Simple greedy movement toward goal
+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X,Y)
    & NX > X & (X1 = X + 1) & not obstacle(X1, Y)
<-  .print("(Greedy) toward ", C, " via right");
    !go(right).

+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X,Y)
    & NX < X & (X1 = X - 1) & not obstacle(X1, Y)
<-  .print("(Greedy) toward ", C, " via left");
    !go(left).

+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X,Y)
    & NY > Y & (Y1 = Y + 1) & not obstacle(X, Y1)
<-  .print("(Greedy) toward ", C, " via down");
    !go(down).

+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X,Y)
    & NY < Y & (Y1 = Y - 1) & not obstacle(X, Y1)
<-  .print("(Greedy) toward ", C, " via up");
    !go(up).

// Obstacle avoidance when direct path blocked
+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X,Y)
    & NX > X & (X1 = X + 1) & obstacle(X1, Y) & NY > Y
    & (Y1 = Y + 1) & not obstacle(X, Y1)
<-  .print("(Greedy dodge) toward ", C, " via down");
    !go(down).

+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X,Y)
    & NX > X & (X1 = X + 1) & obstacle(X1, Y) & NY < Y
    & (Y1 = Y - 1) & not obstacle(X, Y1)
<-  .print("(Greedy dodge) toward ", C, " via up");
    !go(up).

+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X,Y)
    & NY > Y & (Y1 = Y + 1) & obstacle(X, Y1) & NX > X
    & (X1 = X + 1) & not obstacle(X1, Y)
<-  .print("(Greedy dodge) toward ", C, " via right");
    !go(right).

+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X,Y)
    & NY > Y & (Y1 = Y + 1) & obstacle(X, Y1) & NX < X
    & (X1 = X - 1) & not obstacle(X1, Y)
<-  .print("(Greedy dodge) toward ", C, " via left");
    !go(left).

// Handle vertical movement when blocked
+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X, Y)
    & NX == X & NY > Y
    & (Y1 = Y + 1) & obstacle(X, Y1)
    & (Xr = X + 1) & not obstacle(Xr, Y)
<-  .print("(Greedy dodge vertical) toward ", C, " via right");
    !go(right).

+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X, Y)
    & NX == X & NY > Y
    & (Y1 = Y + 1) & obstacle(X, Y1)
    & (Xl = X - 1) & not obstacle(Xl, Y)
<-  .print("(Greedy dodge vertical) toward ", C, " via left");
    !go(left).

+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X, Y)
    & NX == X & NY < Y
    & (Y1 = Y - 1) & obstacle(X, Y1)
    & (Xr = X + 1) & not obstacle(Xr, Y)
<-  .print("(Greedy dodge vertical) toward ", C, " via right");
    !go(right).

+!decide_and_step
    : assigned_goal(C) & goal(C, NX, NY, _R) & agent_position(X, Y)
    & NX == X & NY < Y
    & (Y1 = Y - 1) & obstacle(X, Y1)
    & (Xl = X - 1) & not obstacle(Xl, Y)
<-  .print("(Greedy dodge vertical) toward ", C, " via left");
    !go(left).

// Fallback when no movement rule applies
+!decide_and_step : assigned_goal(_) <- .wait(100).

// Execute movement actions in environment
+!go(up)    <- move(up).
+!go(down)  <- move(down).
+!go(left)  <- move(left).
+!go(right) <- move(right).

// Handle goal collection and notify manager
+received_reward(C, R, At) : assigned_goal(C) <-
    .print("Collected ", C, " (", R, ") at t=", At);
    manager(M);
    .send(M, tell, completed(C));
    -assigned_goal(C).

// Reset agent state between episodes
+episode_end(_,_,_,_) <- -assigned_goal(_).

+time(0)[source(percept)] <- -assigned_goal(_).

+received_reward(C, R, At) : not assigned_goal(C) <-
    .print("Collected ", C, " (", R, ") at t=", At).

+episode_end(TotalR, TotalC, Score, Ep) <-
    .print("EPISODE ", Ep, " END  reward=", TotalR, " cost=", TotalC, " score=", Score).

// Handle collision conflicts with other agents
+conflict(Other) <-
    .print("Conflict with ", Other, "; replanning...").

// Process contract net protocol messages from manager
+cfp(Color, _X, _Y, _R)[source(Mgr)] : not assigned_goal(Color) <-
    !respond_cfp(Mgr, Color, 0).

+!respond_cfp(Mgr, Color, _T)
    : candidate(Color, _NX, _NY, Steps, Net)
<-  .print("Received CFP for ", Color, " (steps=", Steps, ", net=", Net, ")");
    .send(Mgr, tell, propose(Color, Steps, Net)).

+!respond_cfp(Mgr, Color, _T)
    : candidate(Color, _NX, _NY, Steps, Net, _Detour)
<-  .print("Received CFP for ", Color, " (steps=", Steps, ", net=", Net, ") via detour info");
    .send(Mgr, tell, propose(Color, Steps, Net)).

+!respond_cfp(Mgr, Color, T)
    : T < 5 & not candidate(Color, _A, _B, _S, _N) & not candidate(Color, _A2, _B2, _S2, _N2, _D2)
<-  .wait(200);
    T1 = T + 1;
    !respond_cfp(Mgr, Color, T1).

+!respond_cfp(Mgr, Color, T)
    : T >= 5
<-  .print("Received CFP for ", Color, " but no candidate available after retries; proposing default");
    .send(Mgr, tell, propose(Color, 1000, -1.0)).

// Handle auction results from manager
+accept(Color)[source(Mgr)] <-
    .print("Accepted for goal ", Color);
    -assigned_goal(_);
    +assigned_goal(Color).

+reject(Color)[source(Mgr)] <-
    .print("Rejected for goal ", Color).
