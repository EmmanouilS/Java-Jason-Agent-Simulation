package example;
import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.Shape;
import java.awt.Stroke;
import java.awt.geom.Ellipse2D;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.ThreadLocalRandom;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import jason.asSyntax.Literal;
import jason.asSyntax.Structure;
import jason.environment.Environment;

// This class implements a multi-agent environment for testing contract net protocols
// where agents compete for goals in a grid world with collision detection
public class Env extends Environment {

    // Environment configuration constants
    public static final int GRID_SIZE = 9;
    public static final int MAX_MOVES = 31;
    public static final double MOVE_COST = 0.01;

    // GUI rendering parameters
    private static final int CELL = 56;
    private static final int PADDING = 24;

    // Planning algorithm parameters for lookahead evaluation
    private static final double GAMMA = 0.7;
    private static final double HORIZON_PEN = 6.0;

    // Reward mapping for different goal types
    private static final Map<String, Double> REWARD;
    static {
        Map<String, Double> tmp = new HashMap<>();
        tmp.put("green", 0.8);
        tmp.put("yellow", 0.3);
        tmp.put("blue", 0.6);
        tmp.put("purple", 0.2);
        REWARD = Collections.unmodifiableMap(tmp);
    }

    // World state shared across all agents
    private final Set<Cell> obstacles = new HashSet<>();
    private final Map<String, Goal> goals = new HashMap<>();

    // Random number generator for procedural world generation
    private final Random rng = ThreadLocalRandom.current();

    // Per-agent state tracking for independent episode management
    private final Map<String, AgentState> agents = new HashMap<>();

    // Collision detection requires tracking previous positions
    private final Map<String, Cell> lastPositions = new HashMap<>();

    // GUI components for visualization
    private JFrame frame;
    private GridPanel panel;
    private volatile int stepDelayMs = 300;

    private boolean obstaclesInitialized = false;
    
    // Performance logging for analysis
    private static final String SCORE_LOG_FILE = "agent_scores.txt";
    private boolean scoreLogInitialized = false;
    private final Map<String, Double> episodeScores = new HashMap<>();
    private int currentEpisode = 1;
    
    // Prevents duplicate time signals to manager
    private int lastManagerTime = -1;

    // Data structures for representing world elements
    private static class Cell {
        final int x, y;
        Cell(int x, int y) { this.x = x; this.y = y; }
        @Override
        public boolean equals(Object o) {
            if (!(o instanceof Cell)) return false;
            Cell c = (Cell) o;
            return x == c.x && y == c.y;
        }
        @Override
        public int hashCode() { return Arrays.hashCode(new int[]{x, y}); }
        @Override
        public String toString() { return "(" + x + "," + y + ")"; }
    }

    private static class Goal {
        final String color;
        Cell pos;
        final double reward;
        Goal(String color, Cell pos, double reward) {
            this.color = color;
            this.pos = pos;
            this.reward = reward;
        }
    }

    // Tracks individual agent progress through episodes
    private static class AgentState {
        Cell pos;
        int time;
        int episode;
        double totalReward;
        double totalCost;
        List<Cell> lastBestPath = new ArrayList<>();
    }

    // Represents a potential goal with path planning information
    private static class Candidate {
        final String color;
        final Cell next;
        final int steps;
        final double net;
        final List<Cell> path;
        double eval;
        final String detourColor;
        final double detourNet;
        final int detourSteps;
        private Candidate(String color, Cell next, int steps, double net, List<Cell> path,
                          String detourColor, double detourNet, int detourSteps) {
            this.color = color;
            this.next = next;
            this.steps = steps;
            this.net = net;
            this.path = path;
            this.detourColor = detourColor;
            this.detourNet = detourNet;
            this.detourSteps = detourSteps;
        }
        static Candidate reachable(String color, Cell next, int steps, double net, List<Cell> path) {
            return new Candidate(color, next, steps, net, path, null, 0.0, 0);
        }
        static Candidate reachableWithDetour(String color, Cell next, int steps, double net, List<Cell> path,
                                             String detourColor, double detourNet, int detourSteps) {
            return new Candidate(color, next, steps, net, path, detourColor, detourNet, detourSteps);
        }
        static Candidate unreachable(Goal g) {
            return new Candidate(g.color, g.pos, -1, -1.0e9, new ArrayList<>(), null, 0.0, 0);
        }
    }

    private static class DetourInfo {
        final String color;
        final double netGain;
        final int extraSteps;
        final Cell detourGoal;
        DetourInfo(String color, double netGain, int extraSteps, Cell detourGoal) {
            this.color = color;
            this.netGain = netGain;
            this.extraSteps = extraSteps;
            this.detourGoal = detourGoal;
        }
    }

    private static class PathResult {
        final boolean reachable;
        final List<Cell> path;
        PathResult(boolean reachable, List<Cell> path) {
            this.reachable = reachable;
            this.path = path;
        }
    }

    // Core environment interface methods
    @Override
    public synchronized void init(String[] args) {
        parseArgs(args);
        // Initialize world obstacles only once
        if (!obstaclesInitialized) {
            placeObstacles(4);
            obstaclesInitialized = true;
        }
        // Generate random goal placement
        placeAllGoalsRandom();
        // Create agent instances with random starting positions
        agents.clear();
        String[] agentNames = {"bob", "alice"};
        for (String agName : agentNames) {
            AgentState st = new AgentState();
            st.pos = randomFreeCell();
            st.time = 0;
            st.episode = 1;
            st.totalReward = 0.0;
            st.totalCost = 0.0;
            st.lastBestPath = new ArrayList<>();
            agents.put(agName, st);
            lastPositions.put(agName, st.pos);
        }
        ensureGUI();
        initializeScoreLog();
        addBasePerceptsForAllAgentsAndManager();
    }

    @Override
    public synchronized boolean executeAction(String agName, Structure action) {
        // Clear old percepts before processing action
        for (String ag : agents.keySet()) clearPercepts(ag);
        clearPercepts("manager");

        // Execute the requested action
        AgentState st = agents.get(agName);
        if (st == null) return false;
        if ("move".equals(action.getFunctor()) && action.getArity() == 1) {
            String dir = stripQuotes(action.getTerm(0).toString());
            performMove(agName, dir);
        }

        // Update all agents with new world state
        addBasePerceptsForAllAgentsAndManager();

        repaintAsync();
        try { Thread.sleep(stepDelayMs); } catch (InterruptedException ignore) { }
        return true;
    }

    // Core movement and collision handling logic
    private boolean performMove(String agName, String dir) {
        AgentState st = agents.get(agName);
        if (st == null) return false;
        // Store position for potential rollback on collision
        lastPositions.put(agName, st.pos);
        // Always increment counters regardless of move success
        st.time = Math.min(MAX_MOVES, st.time + 1);
        st.totalCost += MOVE_COST;
        // Calculate target position
        Cell next = stepFrom(st.pos, dir);
        boolean valid = isInside(next) && !obstacles.contains(next);
        if (valid) {
            // Detect simultaneous movement conflicts
            String conflictWith = null;
            for (Map.Entry<String, AgentState> e : agents.entrySet()) {
                String other = e.getKey();
                if (other.equals(agName)) continue;
                Cell otherPos = e.getValue().pos;
                // Check for direct position conflicts
                if (next.equals(otherPos)) {
                    conflictWith = other;
                    break;
                }
                // Check for simultaneous movement to same cell
                Cell otherLast = lastPositions.get(other);
                if (otherLast != null && next.equals(e.getValue().pos) && !otherLast.equals(e.getValue().pos)) {
                    conflictWith = other;
                    break;
                }
            }
            if (conflictWith != null) {
                // Rollback both agents to prevent collision
                AgentState otherSt = agents.get(conflictWith);
                Cell otherLast = lastPositions.get(conflictWith);
                if (otherSt != null && otherLast != null) {
                    otherSt.pos = otherLast;
                }
                Cell myLast = lastPositions.get(agName);
                if (myLast != null) {
                    st.pos = myLast;
                }
                // Notify agents of conflict for replanning
                Literal conf1 = Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                        "conflict(%s)", conflictWith));
                Literal conf2 = Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                        "conflict(%s)", agName));
                addPercept(agName, conf1);
                addPercept(conflictWith, conf2);
            } else {
                // Successful movement
                st.pos = next;
                // Check for goal collection and reward
                Goal collected = goalAt(next);
                if (collected != null) {
                    st.totalReward += collected.reward;
                    addPercept(agName, Literal.parseLiteral(String.format(
                            java.util.Locale.ROOT,
                            "received_reward(%s,%.2f,%d)",
                            collected.color, collected.reward, st.time
                    )));
                    // Respawn goal to maintain constant goal count
                    collected.pos = randomFreeCell();
                }
            }
        }
        // Handle episode completion for individual agent
        if (st.time >= MAX_MOVES) {
            double score = st.totalReward - st.totalCost;
            
            logAgentScore(agName, score);
            
            addPercept(agName, Literal.parseLiteral(String.format(
                    java.util.Locale.ROOT,
                    "episode_end(%.2f,%.2f,%.2f,%d)",
                    st.totalReward, st.totalCost, score, st.episode
            )));
            st.episode++;
            // Reset agent for next episode
            st.time = 0;
            st.totalReward = 0.0;
            st.totalCost = 0.0;
            st.pos = randomFreeCell();
            st.lastBestPath = new ArrayList<>();
        }
        return true;
    }

    // Generates world state information for all agents
    private void addBasePerceptsForAllAgentsAndManager() {
        // Generate personalized world view for each agent
        for (Map.Entry<String, AgentState> e : agents.entrySet()) {
            String ag = e.getKey();
            AgentState st = e.getValue();
            int movesLeft = (MAX_MOVES - st.time);
            // Send basic environment information
            addPercept(ag, Literal.parseLiteral("grid_size(" + GRID_SIZE + ")"));
            addPercept(ag, Literal.parseLiteral("move_cost(" + format2(MOVE_COST) + ")"));
            addPercept(ag, Literal.parseLiteral("time(" + st.time + ")"));
            addPercept(ag, Literal.parseLiteral("moves_left(" + movesLeft + ")"));
            addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                    "total_reward(%.2f)", st.totalReward)));
            addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                    "total_cost(%.2f)", st.totalCost)));
            addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                    "score(%.2f)", (st.totalReward - st.totalCost))));
            addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                    "agent_position(%d,%d)", st.pos.x, st.pos.y)));
            // Send obstacle information
            for (Cell o : obstacles) {
                addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                        "obstacle(%d,%d)", o.x, o.y)));
            }
            // Generate goal information and path planning
            List<Candidate> cands = new ArrayList<>();
            for (Goal g : goals.values()) {
                // Send goal location to agent
                addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                        "goal(%s,%d,%d,%.2f)",
                        g.color, g.pos.x, g.pos.y, g.reward)));
                // Notify manager to start contract net protocol
                addPercept("manager", Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                        "goal(%s,%d,%d,%.2f)",
                        g.color, g.pos.x, g.pos.y, g.reward)));
                // Plan path using A* search
                PathResult pr = aStar(st.pos, g.pos);
                if (!pr.reachable) {
                    cands.add(Candidate.unreachable(g));
                    continue;
                }
                // Check for profitable detour opportunities
                DetourInfo detour = findBestDetour(st.pos, g.pos, pr.path, goals);
                if (detour != null) {
                    List<Cell> detourPath = createPathWithDetour(st.pos, g.pos, detour);
                    int detourSteps = detourPath.size() - 1;
                    double detourNet = g.reward + detour.netGain - detourSteps * MOVE_COST;
                    Cell next = (detourPath.size() > 1) ? detourPath.get(1) : st.pos;
                    cands.add(Candidate.reachableWithDetour(g.color, next, detourSteps, detourNet,
                            detourPath, detour.color, detour.netGain, detour.extraSteps));
                } else {
                    int steps = pr.path.size() - 1;
                    double net = g.reward - steps * MOVE_COST;
                    Cell next = (pr.path.size() > 1) ? pr.path.get(1) : st.pos;
                    cands.add(Candidate.reachable(g.color, next, steps, net, pr.path));
                }
            }
            // Apply lookahead evaluation to rank candidates
            for (Candidate c : cands) {
                if (c.steps < 0) { c.eval = -1.0e9; continue; }
                double best2Net = 0.0;
                if (c.steps < movesLeft) {
                    // Simulate future goal collection after this one
                    Cell posAfterC = goals.get(c.color).pos;
                    for (Goal g2 : goals.values()) {
                        if (g2.color.equals(c.color)) continue;
                        PathResult pr2 = aStar(posAfterC, g2.pos);
                        if (!pr2.reachable) continue;
                        int s2 = pr2.path.size() - 1;
                        double n2 = g2.reward - s2 * MOVE_COST;
                        if (n2 > best2Net) best2Net = n2;
                    }
                }
                double horizonPen = (c.steps > movesLeft) ? HORIZON_PEN * (c.steps - movesLeft) * MOVE_COST : 0.0;
                c.eval = c.net + GAMMA * best2Net - horizonPen;
            }
            // Rank candidates by evaluation score
            if (!cands.isEmpty()) {
                cands.sort((a, b) -> {
                    int cmp = Double.compare(b.eval, a.eval);
                    if (cmp != 0) return cmp;
                    int sa = (a.steps < 0 ? Integer.MAX_VALUE : a.steps);
                    int sb = (b.steps < 0 ? Integer.MAX_VALUE : b.steps);
                    return Integer.compare(sa, sb);
                });
                Candidate best = cands.get(0);
                if (best.steps >= 0) st.lastBestPath = best.path; else st.lastBestPath = new ArrayList<>();
                addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                        "best_target(%s,%d,%d,%d,%.2f)",
                        best.color, best.next.x, best.next.y, best.steps, best.net)));
                // Send all candidate information to agent
                for (Candidate c : cands) {
                    if (c.steps >= 0) {
                        if (c.detourColor != null) {
                            addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                                    "candidate(%s,%d,%d,%d,%.2f,detour(%s,%.2f,%d))",
                                    c.color, c.next.x, c.next.y, c.steps, c.net,
                                    c.detourColor, c.detourNet, c.detourSteps)));
                        } else {
                            addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                                    "candidate(%s,%d,%d,%d,%.2f)",
                                    c.color, c.next.x, c.next.y, c.steps, c.net)));
                        }
                    } else {
                        Goal g = goals.get(c.color);
                        addPercept(ag, Literal.parseLiteral(String.format(java.util.Locale.ROOT,
                                "candidate(%s,%d,%d,%d,%s)",
                                c.color, g.pos.x, g.pos.y, -1, "-1.0e9")));
                    }
                }
            } else {
                st.lastBestPath = new ArrayList<>();
            }
        }
        
        // Synchronize episode timing across all agents
        boolean allAgentsCompleted = agents.values().stream()
                .allMatch(st -> st.time == 0 && st.episode > 1);

        int minPositiveTime = agents.values().stream()
                .mapToInt(st -> st.time)
                .filter(t -> t > 0)
                .min()
                .orElse(1);

        if (allAgentsCompleted) {
            // Reset world when all agents complete episodes
            resetEnvironmentForNewEpisode();
            if (lastManagerTime != 0) {
                addPercept("manager", Literal.parseLiteral("time(0)"));
                lastManagerTime = 0;
            }
        } else {
            if (lastManagerTime != minPositiveTime) {
                addPercept("manager", Literal.parseLiteral("time(" + minPositiveTime + ")"));
                lastManagerTime = minPositiveTime;
            }
        }
    }

    // A* pathfinding algorithm for optimal path planning
    private PathResult aStar(Cell start, Cell goal) {
        if (start.equals(goal)) {
            return new PathResult(true, Arrays.asList(start));
        }
        PriorityQueue<Node> open = new PriorityQueue<>(11, (a, b) -> Double.compare(a.f, b.f));
        Map<Cell, Double> gScore = new HashMap<>();
        Set<Cell> closed = new HashSet<>();
        Node startNode = new Node(start, 0.0, heuristic(start, goal), null);
        open.add(startNode);
        gScore.put(start, 0.0);
        while (!open.isEmpty()) {
            Node current = open.poll();
            if (current.c.equals(goal)) {
                List<Cell> path = new ArrayList<>();
                Node n = current;
                while (n != null) { path.add(n.c); n = n.parent; }
                Collections.reverse(path);
                return new PathResult(true, path);
            }
            closed.add(current.c);
            for (Cell nb : neighbors(current.c)) {
                if (!isInside(nb) || obstacles.contains(nb)) continue;
                if (closed.contains(nb)) continue;
                double tentativeG = current.g + 1.0;
                Double bestG = gScore.get(nb);
                if (bestG == null || tentativeG < bestG) {
                    Node nbNode = new Node(nb, tentativeG, tentativeG + heuristic(nb, goal), current);
                    gScore.put(nb, tentativeG);
                    open.add(nbNode);
                }
            }
        }
        return new PathResult(false, new ArrayList<>());
    }
    private static class Node {
        final Cell c;
        final double g, f;
        final Node parent;
        Node(Cell c, double g, double f, Node parent) {
            this.c = c;
            this.g = g;
            this.f = f;
            this.parent = parent;
        }
    }
    private List<Cell> neighbors(Cell c) {
        return Arrays.asList(
                new Cell(c.x + 1, c.y),
                new Cell(c.x - 1, c.y),
                new Cell(c.x, c.y + 1),
                new Cell(c.x, c.y - 1)
        );
    }
    private static int heuristic(Cell a, Cell b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    // Detour detection for multi-goal optimization
    private DetourInfo findBestDetour(Cell start, Cell target, List<Cell> path, Map<String, Goal> goals) {
        DetourInfo bestDetour = null;
        double bestNetGain = 0.0;
        for (Goal g : goals.values()) {
            if (g.pos.equals(target)) continue;
            boolean isNearPath = false;
            for (Cell pathCell : path) {
                if (heuristic(pathCell, g.pos) <= 2) {
                    isNearPath = true;
                    break;
                }
            }
            if (!isNearPath) continue;
            PathResult toDetour = aStar(start, g.pos);
            PathResult fromDetour = aStar(g.pos, target);
            if (!toDetour.reachable || !fromDetour.reachable) continue;
            int directSteps = path.size() - 1;
            int detourSteps = (toDetour.path.size() - 1) + (fromDetour.path.size() - 1);
            int extraSteps = detourSteps - directSteps;
            double netGain = g.reward - 0.01 * extraSteps;
            if (netGain > 0 && netGain > bestNetGain) {
                bestDetour = new DetourInfo(g.color, netGain, extraSteps, g.pos);
                bestNetGain = netGain;
            }
        }
        return bestDetour;
    }
    private List<Cell> createPathWithDetour(Cell start, Cell target, DetourInfo detour) {
        if (detour == null) {
            PathResult pr = aStar(start, target);
            return pr.reachable ? pr.path : new ArrayList<>();
        }
        PathResult toDetour = aStar(start, detour.detourGoal);
        PathResult fromDetour = aStar(detour.detourGoal, target);
        if (!toDetour.reachable || !fromDetour.reachable) {
            PathResult pr = aStar(start, target);
            return pr.reachable ? pr.path : new ArrayList<>();
        }
        List<Cell> combined = new ArrayList<>(toDetour.path);
        if (fromDetour.path.size() > 1) {
            combined.addAll(fromDetour.path.subList(1, fromDetour.path.size()));
        }
        return combined;
    }

    // Episode reset functionality
    private void resetEnvironmentForNewEpisode() {
        // Regenerate world obstacles
        obstacles.clear();
        placeObstacles(4);
        
        // Regenerate goal positions
        goals.clear();
        placeAllGoalsRandom();
        
        // Reset all agent states
        for (Map.Entry<String, AgentState> e : agents.entrySet()) {
            String agName = e.getKey();
            AgentState st = e.getValue();
            
            st.pos = randomFreeCell();
            st.time = 0;
            st.totalReward = 0.0;
            st.totalCost = 0.0;
            st.lastBestPath = new ArrayList<>();
            
            lastPositions.put(agName, st.pos);
        }
    }

    // World generation and placement methods
    private void placeObstacles(int count) {
        obstacles.clear();
        while (obstacles.size() < count) {
            obstacles.add(new Cell(rng.nextInt(GRID_SIZE), rng.nextInt(GRID_SIZE)));
        }
    }
    private void placeAllGoalsRandom() {
        goals.clear();
        placeGoal("green");
        placeGoal("yellow");
        placeGoal("blue");
        placeGoal("purple");
    }
    private void placeGoal(String color) {
        goals.put(color, new Goal(color, randomFreeCell(), REWARD.get(color)));
    }
    private Cell randomFreeCell() {
        Cell c;
        int attempts = 0;
        do {
            c = new Cell(rng.nextInt(GRID_SIZE), rng.nextInt(GRID_SIZE));
            attempts++;
            if (attempts > 10000) throw new IllegalStateException("No free cell");
        } while (isOccupied(c));
        return c;
    }
    private boolean isOccupied(Cell c) {
        if (obstacles.contains(c)) return true;
        for (AgentState st : agents.values()) {
            if (st.pos != null && st.pos.equals(c)) return true;
        }
        for (Goal g : goals.values()) if (g.pos.equals(c)) return true;
        return false;
    }
    private boolean isInside(Cell c) {
        return c.x >= 0 && c.x < GRID_SIZE && c.y >= 0 && c.y < GRID_SIZE;
    }
    private Goal goalAt(Cell c) {
        for (Goal g : goals.values()) if (g.pos.equals(c)) return g;
        return null;
    }
    private Cell stepFrom(Cell c, String dir) {
        int nx = c.x;
        int ny = c.y;
        switch (dir) {
            case "up" -> ny--;
            case "down" -> ny++;
            case "left" -> nx--;
            case "right" -> nx++;
        }
        return new Cell(nx, ny);
    }

    // Graphical user interface components
    private void ensureGUI() {
        if (GraphicsEnvironment.isHeadless()) return;
        if (frame != null) return;
        SwingUtilities.invokeLater(() -> {
            frame = new JFrame("MultiAgentEnv — 9×9 Grid");
            frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
            panel = new GridPanel();
            JPanel root = new JPanel(new BorderLayout());
            root.add(panel, BorderLayout.CENTER);
            root.add(new ControlPanel(), BorderLayout.SOUTH);
            frame.setContentPane(root);
            frame.pack();
            frame.setLocationByPlatform(true);
            frame.setVisible(true);
        });
    }
    private void repaintAsync() {
        if (panel == null) return;
        SwingUtilities.invokeLater(() -> panel.repaint());
    }

    private Snapshot snapshot() {
        Map<String, GoalCopy> gs = new HashMap<>();
        for (Map.Entry<String, Goal> e : goals.entrySet()) {
            Goal g = e.getValue();
            gs.put(e.getKey(), new GoalCopy(g.color, g.pos.x, g.pos.y, g.reward));
        }
        Set<Cell> obs = new HashSet<>(obstacles);
        Map<String, Cell> as = new HashMap<>();
        for (Map.Entry<String, AgentState> e : agents.entrySet()) {
            as.put(e.getKey(), new Cell(e.getValue().pos.x, e.getValue().pos.y));
        }
        Map<String, List<Cell>> paths = new HashMap<>();
        for (Map.Entry<String, AgentState> e : agents.entrySet()) {
            paths.put(e.getKey(), new ArrayList<>(e.getValue().lastBestPath));
        }
        return new Snapshot(GRID_SIZE, as, obs, gs, paths);
    }
    private static class GoalCopy {
        final String color; final int x, y; final double r;
        GoalCopy(String c, int x, int y, double r) { this.color = c; this.x = x; this.y = y; this.r = r; }
    }
    private static class Snapshot {
        final int size;
        final Map<String, Cell> agents;
        final Set<Cell> obstacles;
        final Map<String, GoalCopy> goals;
        final Map<String, List<Cell>> bestPaths;
        Snapshot(int size, Map<String, Cell> agents, Set<Cell> obstacles,
                 Map<String, GoalCopy> goals, Map<String, List<Cell>> bestPaths) {
            this.size = size;
            this.agents = agents;
            this.obstacles = obstacles;
            this.goals = goals;
            this.bestPaths = bestPaths;
        }
    }
    private class GridPanel extends JPanel {
        GridPanel() {
            setPreferredSize(new Dimension(PADDING*2 + CELL*GRID_SIZE, PADDING*2 + CELL*GRID_SIZE + 200));
            setBackground(Color.white);
        }
        @Override
        protected void paintComponent(Graphics g0) {
            super.paintComponent(g0);
            Graphics2D g = (Graphics2D) g0;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            Snapshot s;
            synchronized (Env.this) { s = snapshot(); }
            // Grid background
            g.setColor(new Color(248, 248, 248));
            g.fillRoundRect(PADDING-6, PADDING-6, CELL*s.size+12, CELL*s.size+12, 12, 12);
            // Grid lines
            g.setColor(new Color(210, 210, 210));
            for (int i = 0; i <= s.size; i++) {
                int x = PADDING + i*CELL;
                int y = PADDING + i*CELL;
                g.drawLine(PADDING, y, PADDING + CELL*s.size, y);
                g.drawLine(x, PADDING, x, PADDING + CELL*s.size);
            }
            // Obstacles
            g.setColor(Color.black);
            for (Cell o : s.obstacles) {
                int x = PADDING + o.x*CELL;
                int y = PADDING + o.y*CELL;
                g.fillRoundRect(x+4, y+4, CELL-8, CELL-8, 10, 10);
            }
            // Best paths: draw for each agent a dashed line in a unique colour
            Stroke oldStroke = g.getStroke();
            float[] dash = {6f, 6f};
            g.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10f, dash, 0f));
            Color[] pathColours = { new Color(80, 80, 200), new Color(200, 80, 80), new Color(80, 200, 80), new Color(200, 200, 80) };
            int idx = 0;
            for (Map.Entry<String, List<Cell>> entry : s.bestPaths.entrySet()) {
                List<Cell> path = entry.getValue();
                if (path == null || path.size() <= 1) continue;
                g.setColor(pathColours[idx % pathColours.length]);
                for (int i = 0; i < path.size()-1; i++) {
                    Cell a = path.get(i);
                    Cell b = path.get(i+1);
                    int ax = PADDING + a.x*CELL + CELL/2;
                    int ay = PADDING + a.y*CELL + CELL/2;
                    int bx = PADDING + b.x*CELL + CELL/2;
                    int by = PADDING + b.y*CELL + CELL/2;
                    g.drawLine(ax, ay, bx, by);
                }
                idx++;
            }
            g.setStroke(oldStroke);
            // Goals
            for (GoalCopy gc : s.goals.values()) {
                Color col = goalColor(gc.color);
                g.setColor(col);
                int x = PADDING + gc.x*CELL;
                int y = PADDING + gc.y*CELL;
                int d = CELL - 12;
                Shape oval = new Ellipse2D.Double(x+6, y+6, d, d);
                g.fill(oval);
                g.setColor(new Color(0, 0, 0, 120));
                g.draw(oval);
                g.setFont(g.getFont().deriveFont(Font.BOLD, 12f));
                String lab = gc.color + "=" + format2(gc.r);
                drawCenteredString(g, lab, new Rectangle(x, y, CELL, CELL), g.getFont());
            }
            // Agents
            int agentIndex = 0;
            Color[] agentColours = { new Color(220, 60, 50), new Color(60, 160, 220), new Color(60, 200, 80), new Color(200, 80, 160) };
            for (Map.Entry<String, Cell> e : s.agents.entrySet()) {
                Cell p = e.getValue();
                int x = PADDING + p.x*CELL;
                int y = PADDING + p.y*CELL;
                int d = CELL - 18;
                Color agCol = agentColours[agentIndex % agentColours.length];
                g.setColor(agCol);
                g.fill(new Ellipse2D.Double(x+9, y+9, d, d));
                g.setColor(Color.white);
                g.setStroke(new BasicStroke(2f));
                g.drawOval(x+9, y+9, d, d);
                agentIndex++;
            }
            // HUD: show agent status information
            int hudY = PADDING*2 + CELL*s.size + 12;
            g.setColor(new Color(30, 30, 30));
            g.setFont(g.getFont().deriveFont(Font.PLAIN, 14f));
            g.drawString("Multi‑Agent Environment", PADDING, hudY);
            
            // Show agent status information
            int statusY = hudY + 25;
            g.setFont(g.getFont().deriveFont(Font.BOLD, 12f));
            g.setColor(new Color(50, 50, 50));
            
            // Get current agent states for display
            Map<String, AgentState> currentStates;
            synchronized (Env.this) { 
                currentStates = new HashMap<>(agents);
            }
            
            agentIndex = 0;
            
            for (Map.Entry<String, AgentState> e : currentStates.entrySet()) {
                String agentName = e.getKey();
                AgentState state = e.getValue();
                
                Color agCol = agentColours[agentIndex % agentColours.length];
                g.setColor(agCol);
                
                int x = PADDING + (agentIndex * 200);
                int y = statusY;
                
                // Agent name
                g.drawString(agentName.toUpperCase() + ":", x, y);
                
                // Steps remaining
                g.setColor(new Color(70, 70, 70));
                g.setFont(g.getFont().deriveFont(Font.PLAIN, 11f));
                g.drawString("Steps: " + state.time + "/" + MAX_MOVES, x, y + 15);
                
                // Episode
                g.drawString("Episode: " + state.episode, x, y + 30);
                
                // Score (reward - cost)
                double score = state.totalReward - state.totalCost;
                g.drawString("Score: " + format2(score), x, y + 45);
                
                // Reward and Cost breakdown
                g.drawString("Reward: " + format2(state.totalReward), x, y + 60);
                g.drawString("Cost: " + format2(state.totalCost), x, y + 75);
                
                agentIndex++;
            }
        }
        private void drawCenteredString(Graphics2D g, String text, Rectangle rect, Font font) {
            FontMetrics m = g.getFontMetrics(font);
            int x = rect.x + (rect.width - m.stringWidth(text)) / 2;
            int y = rect.y + ((rect.height - m.getHeight()) / 2) + m.getAscent();
            g.setFont(font);
            g.setColor(Color.black);
            g.drawString(text, x+1, y+1);
            g.setColor(Color.white);
            g.drawString(text, x, y);
        }
    }
    private class ControlPanel extends JPanel {
        private final JLabel label;
        private final JSlider slider;
        ControlPanel() {
            setLayout(new FlowLayout(FlowLayout.LEFT));
            label = new JLabel("Delay: " + stepDelayMs + " ms");
            slider = new JSlider(0, 1000, stepDelayMs);
            slider.setMajorTickSpacing(250);
            slider.setMinorTickSpacing(50);
            slider.setPaintTicks(true);
            slider.addChangeListener(e -> {
                stepDelayMs = slider.getValue();
                label.setText("Delay: " + stepDelayMs + " ms");
            });
            add(label);
            add(slider);
        }
    }

    // Performance logging and analysis
    private void initializeScoreLog() {
        if (scoreLogInitialized) return;
        try (FileWriter writer = new FileWriter(SCORE_LOG_FILE, false)) {
            writer.write("Episode,Bob Score,Alice Score\n");
            scoreLogInitialized = true;
        } catch (IOException e) {
            System.err.println("Failed to initialize score log: " + e.getMessage());
        }
    }
    
    private void logAgentScore(String agentName, double score) {
        // Store the score for this agent
        episodeScores.put(agentName, score);
        
        // Check if both agents have completed this episode
        if (episodeScores.containsKey("bob") && episodeScores.containsKey("alice")) {
            try (FileWriter writer = new FileWriter(SCORE_LOG_FILE, true)) {
                double bobScore = episodeScores.getOrDefault("bob", 0.0);
                double aliceScore = episodeScores.getOrDefault("alice", 0.0);
                writer.write(String.format("%d,%.3f,%.3f%n", currentEpisode, bobScore, aliceScore));
                
                // Clear scores and move to next episode
                episodeScores.clear();
                currentEpisode++;
            } catch (IOException e) {
                System.err.println("Failed to log episode scores: " + e.getMessage());
            }
        }
    }

    // Utility methods and helpers
    private void parseArgs(String[] args) {
        if (args == null) return;
        for (String a : args) {
            if (a == null) continue;
            String s = a.trim();
            try {
                if (s.startsWith("delay=")) {
                    stepDelayMs = Integer.parseInt(s.substring(6));
                } else if (s.matches("^\\d+$")) {
                    stepDelayMs = Integer.parseInt(s);
                }
            } catch (NumberFormatException ignore) { }
        }
    }
    private static String stripQuotes(String s) {
        if (s == null) return null;
        int n = s.length();
        if (n >= 2 && ((s.charAt(0) == '\'' && s.charAt(n-1) == '\'') || (s.charAt(0) == '"' && s.charAt(n-1) == '"'))) {
            return s.substring(1, n-1);
        }
        return s;
    }
    private static String format2(double v) {
        return String.format(java.util.Locale.ROOT, "%.2f", v);
    }
    private static Color goalColor(String name) {
        return switch (name) {
            case "green" -> new Color(40, 170, 80);
            case "yellow" -> new Color(220, 190, 40);
            case "blue" -> new Color(70, 120, 220);
            case "purple" -> new Color(150, 70, 190);
            default -> new Color(100, 100, 100);
        };
    }
}