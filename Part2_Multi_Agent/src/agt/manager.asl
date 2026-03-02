// Contract Net Protocol manager for task allocation

// Define available worker agents
worker(bob).
worker(alice).

// Global lock to prevent race conditions in selection
selecting_any(false).

// Reset manager state when new episode begins
+time(0)[source(percept)] <- !reset_manager.

+!reset_manager <-
    -assigned(_, _);
    -busy(_);
    -auction_running(_);
    -expected(_, _);
    -proposal(_, _, _, _);
    -selecting(_);
    -selecting_any(_);
    -goal(_,_,_,_);
    -goal_info(_,_,_,_);
    +selecting_any(false);
    .print("Manager: state cleared for new episode.");
    .wait(1);
    !rescan_goals.

// Start auctions for all available goals
+!rescan_goals <-
    .findall([C,X,Y,R], goal(C,X,Y,R), Gs);
    .print("Manager: rescanning goals - found ", Gs);
    !start_auctions(Gs).

+!start_auctions([[C,X,Y,R] | Rest]) : not assigned(C,_) & not auction_running(C) <-
    .print("Manager: (rescan) starting CNP for ", C, " at ", X, ",", Y);
    !contract_net(C,X,Y,R);
    !start_auctions(Rest).

+!start_auctions([_ | Rest]) <- !start_auctions(Rest).

+!start_auctions([]) <- true.

// Clear all proposals for a specific goal color
+!clear_proposals(Color) : proposal(Color, W, S, N) <-
    -proposal(Color, W, S, N);
    !clear_proposals(Color).

+!clear_proposals(Color) : not proposal(Color, _, _, _) <-
    .print("Manager: proposals cleared for ", Color).

// Initialize manager on startup
!start.
+!start <- 
    .print("Manager ready: running contract nets for goals...");
    -selecting_any(_);
    +selecting_any(false).

// Start auction when new goal is discovered
+goal(Color, X, Y, R) : not assigned(Color, _) & not auction_running(Color) <-
    .print("Manager: discovered goal ", Color, " at ", X, ",", Y, " with reward ", R);
    !contract_net(Color, X, Y, R).

// Execute contract net protocol for goal allocation
+!contract_net(Color, X, Y, R) <-
    +auction_running(Color);
    +goal_info(Color, X, Y, R);
    !clear_proposals(Color);
    .findall(W, worker(W), Ws);
    .length(Ws, N);
    +expected(Color, N);
    .wait(150);
    .send(bob,   tell, cfp(Color, X, Y, R));
    .send(alice, tell, cfp(Color, X, Y, R));
    .print("Manager: sent CFP for goal ", Color, " to ", N, " workers").

// Process worker proposals for goal allocation
+propose(Color, Steps, Net)[source(W)]
    : auction_running(Color) & not assigned(Color, _)
<-
    -proposal(Color, W, _, _);
    +proposal(Color, W, Steps, Net);
    ?expected(Color, N);
    .count(proposal(Color, _, _, _), C);
    Left = N - C;
    .print("Manager: received proposal for ", Color, " from ", W,
           " (steps=", Steps, ", net=", Net, ")  [", Left, " left]");
    !check_ready(Color).

// Check if all proposals received and trigger selection
+!check_ready(Color)
    : expected(Color, N)
    & .count(proposal(Color, _, _, _), C)
    & not (N > C)
    & auction_running(Color)
    & not assigned(Color, _)
<-  !select_best_atomic(Color).

+!check_ready(_Color) <- true.

// Atomic selection wrapper to prevent race conditions

+!select_best_atomic(Color)
    : selecting_any(true)
<-  .wait(1);
    !select_best_atomic(Color).

+!select_best_atomic(Color)
    : selecting_any(false)
<-  -selecting_any(false);
    +selecting_any(true);
    !select_best(Color);
    -selecting_any(true);
    +selecting_any(false).

+!select_best_atomic(_Color) <- true.

// Handle selection errors gracefully
+!select_best_atomic(Color)[error(action_failed)] <-
    .print("Manager: selection failed for ", Color, "; cleaning up.");
    -selecting_any(true);
    -selecting(Color);
    -expected(Color, _);
    -auction_running(Color);
    +selecting_any(false).

// Select best worker based on proposal quality and availability
+!select_best(Color) : assigned(Color, _) <-
    .print("Manager: ", Color, " already assigned; ignoring selection.");
    -expected(Color, _);
    -auction_running(Color).

// Accept best available worker for the goal
+!select_best(Color)
    : proposal(Color, W, S, N)
    & not (proposal(Color, _, _, N2) & N2 > N)
    & not (proposal(Color, _, S3, N3) & N3 == N & S3 < S)
    & not busy(W)
<-
    .print("Manager: selecting ", W, " for goal ", Color, " (net=", N, ", steps=", S, ")");
    .send(W, tell, accept(Color));
    +assigned(Color, W);
    +busy(W);
    !clear_proposals(Color);
    -expected(Color, _);
    -auction_running(Color).

// Skip busy workers and try next best option
+!select_best(Color)
    : proposal(Color, W, S, N)
    & not (proposal(Color, _, _, N2) & N2 > N)
    & not (proposal(Color, _, S3, N3) & N3 == N & S3 < S)
    & busy(W)
<-
    .print("Manager: best worker ", W, " for ", Color, " is busy; trying next...");
    -proposal(Color, W, S, N);
    !select_best(Color).

// Close auction if no workers available
+!select_best(Color) : not proposal(Color, _, _, _) <-
    .print("Manager: no free worker for ", Color, "; will retry on next percept.");
    -expected(Color, _);
    -auction_running(Color).

// Free worker when task is completed
+completed(Color)[source(W)] : assigned(Color, W) <-
    .print("Manager: ", W, " completed ", Color, "; freeing worker.");
    -assigned(Color, W);
    -busy(W).
