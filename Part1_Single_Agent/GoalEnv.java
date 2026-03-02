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
import java.awt.geom.Ellipse2D;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
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

public class GoalEnv extends Environment {

    // Environment configuration constants
    public static final int GRID_SIZE = 9;
    public static final int MAX_MOVES = 31;
    public static final double MOVE_COST = 0.01;

    // GUI layout constants
    private static final int CELL = 56;    // pixels per cell
    private static final int PADDING = 24; // padding around grid

    // Animation speed control
    private volatile int stepDelayMs = 300;

    // Decision-making parameters
    private static final double GAMMA = 0.7;           // discount factor for lookahead
    private static final double HORIZON_PEN = 6.0;     // penalty multiplier for over-horizon steps

    // World state variables
    private final Map<String, Goal> goals = new HashMap<>(); // color -> goal mapping
    private final Set<Cell> obstacles = new HashSet<>();     // obstacle positions
    private Cell agent;                                      // current agent position
    private int time;                                        // current time step (0..MAX_MOVES)
    private int episode = 1;                                 // current episode number
    private double totalReward;                             // cumulative reward this episode
    private double totalCost;                               // cumulative movement cost this episode
    private final String logFileName = "episode_rewards.txt"; // log file for episode results

    private final java.util.Random rng = ThreadLocalRandom.current();

    // Goal reward values by color
    private static final Map<String, Double> REWARD;
    static {
        Map<String, Double> m = new HashMap<>();
        m.put("green", 0.8);   // highest value
        m.put("yellow", 0.3);  // medium value
        m.put("blue", 0.6);    // high value
        m.put("purple", 0.2);  // lowest value
        REWARD = Collections.unmodifiableMap(m);
    }

    // Current best path for visualization
    private List<Cell> lastBestPath = new ArrayList<>();

    // GUI components
    private JFrame frame;
    private GridPanel panel;

    /**
     * Represents a cell position in the grid
     */
    private static class Cell {
        final int x, y;
        
        Cell(int x, int y) { 
            this.x = x; 
            this.y = y; 
        }
        
        @Override 
        public boolean equals(Object o) { 
            if (!(o instanceof Cell)) return false; 
            Cell c = (Cell)o; 
            return x==c.x && y==c.y; 
        }
        
        @Override 
        public int hashCode() { 
            return Arrays.hashCode(new int[]{x,y}); 
        }
        
        @Override 
        public String toString() { 
            return "("+x+","+y+")"; 
        }
    }

    /**
     * Represents a goal with color, position, and reward value
     */
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

    // ===== Jason Environment API Implementation =====

    /**
     * Initialize the environment with random goal and obstacle placement
     */
    @Override
    public synchronized void init(String[] args) {
        parseArgs(args);
        initializeLogFile();
        placeObstacles(4);
        placeAllGoalsRandom();
        placeAgentRandom();
        resetEpisodeCounters();
        ensureGUI();
        updatePercepts();
    }

    /**
     * Execute agent actions (currently only move commands)
     */
    @Override
    public synchronized boolean executeAction(String ag, Structure action) {
        String fun = action.getFunctor();
        boolean changed = false;

        if ("move".equals(fun) && action.getArity() == 1) {
            String dir = stripQuotes(action.getTerm(0).toString());
            changed = performMove(dir);
        }

        updatePercepts();
        repaintAsync();
        try { 
            Thread.sleep(stepDelayMs); 
        } catch (InterruptedException ignored) {}
        return changed;
    }

    // ===== Core Environment Logic =====

    /**
     * Reset episode counters and clear the best path
     */
    private void resetEpisodeCounters() {
        time = 0;
        totalReward = 0.0;
        totalCost = 0.0;
        lastBestPath = new ArrayList<>();
    }

    /**
     * Execute a move action and handle goal collection and episode termination
     */
    private boolean performMove(String dir) {
        // Consume time and cost regardless of move validity
        time = Math.min(MAX_MOVES, time + 1);
        totalCost += MOVE_COST;

        // Calculate next position and validate move
        Cell next = stepFrom(agent, dir);
        boolean valid = isInside(next) && !obstacles.contains(next);
        if (valid) agent = next;

        // Check for goal collection
        Goal collected = goalAt(agent);
        if (collected != null) {
            totalReward += collected.reward;
            addPerceptOnce(Literal.parseLiteral(String.format(
                    java.util.Locale.ROOT,
                    "received_reward(%s,%.2f,%d)",
                    collected.color, collected.reward, time
            )));
            // Respawn goal at new random location
            collected.pos = randomFreeCell();
        }

        // Check for episode termination
        if (time >= MAX_MOVES) {
            double finalScore = totalReward - totalCost;
            addPerceptOnce(Literal.parseLiteral(String.format(
                    java.util.Locale.ROOT,
                    "episode_end(%.2f,%.2f,%.2f,%d)",
                    totalReward, totalCost, finalScore, episode
            )));
            
            // Log episode results
            logEpisodeResult(episode, totalReward, totalCost, finalScore, time);
            
            // Start new episode
            episode++;
            placeAllGoalsRandom();
            placeAgentRandom();
            resetEpisodeCounters();
        }
        return true;
    }

    /**
     * Calculate next cell position based on direction
     */
    private Cell stepFrom(Cell c, String dir) {
        int nx = c.x, ny = c.y;
        switch (dir) {
            case "up" -> ny--;
            case "down" -> ny++;
            case "left" -> nx--;
            case "right" -> nx++;
        }
        return new Cell(nx, ny);
    }

    /**
     * Find goal at specified cell position
     */
    private Goal goalAt(Cell c) {
        for (Goal g : goals.values()) 
            if (g.pos.equals(c)) return g;
        return null;
    }

    // ===== Percept Generation and Planning =====

    /**
     * Generate all percepts for the agent including world state and goal evaluations
     */
    private void updatePercepts() {
        clearPercepts();
        int movesLeft = (MAX_MOVES - time);

        // Basic world information
        addPerceptOnce("grid_size(" + GRID_SIZE + ")");
        addPerceptOnce("move_cost(" + format2(MOVE_COST) + ")");
        addPerceptOnce("time(" + time + ")");
        addPerceptOnce("moves_left(" + movesLeft + ")");
        addPerceptOnce(String.format(java.util.Locale.ROOT, "total_reward(%.2f)", totalReward));
        addPerceptOnce(String.format(java.util.Locale.ROOT, "total_cost(%.2f)", totalCost));
        addPerceptOnce(String.format(java.util.Locale.ROOT, "score(%.2f)", (totalReward - totalCost)));
        addPerceptOnce(String.format(java.util.Locale.ROOT, "agent_position(%d,%d)", agent.x, agent.y));

        // Obstacle positions
        for (Cell o : obstacles)
            addPerceptOnce(String.format(java.util.Locale.ROOT, "obstacle(%d,%d)", o.x, o.y));

        // Evaluate all goals as potential targets
        List<Candidate> cands = new ArrayList<>();
        for (Goal g : goals.values()) {
            PathResult pr = aStar(agent, g.pos);
            addPerceptOnce(String.format(java.util.Locale.ROOT, "goal(%s,%d,%d,%.2f)", g.color, g.pos.x, g.pos.y, g.reward));
            
            if (!pr.reachable) {
                cands.add(Candidate.unreachable(g));
                continue;
            }
            
            // Check for profitable detours
            DetourInfo detour = findBestDetour(agent, g.pos, pr.path, goals);
            
            if (detour != null) {
                // Use detour path
                List<Cell> detourPath = createPathWithDetour(agent, g.pos, detour);
                int detourSteps = detourPath.size() - 1;
                double detourNet = g.reward + detour.netGain - detourSteps * MOVE_COST;
                Cell next = (detourPath.size() > 1) ? detourPath.get(1) : agent;
                cands.add(Candidate.reachableWithDetour(g.color, next, detourSteps, detourNet, detourPath, 
                                                       detour.color, detour.netGain, detour.extraSteps));
            } else {
                // Use direct path
                int steps = pr.path.size() - 1;
                double net = g.reward - steps * MOVE_COST;
                Cell next = (pr.path.size() > 1) ? pr.path.get(1) : agent;
                cands.add(Candidate.reachable(g.color, next, steps, net, pr.path));
            }
        }

        // Evaluate candidates with one-step lookahead
        for (Candidate c : cands) {
            if (c.steps < 0) { 
                c.eval = -1.0e9; 
                continue; 
            }
            
            double best2Net = 0.0;
            if (c.steps < movesLeft) {
                // Simulate standing on goal position and find best follow-up goal
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
            
            // Calculate evaluation with horizon penalty
            double horizonPen = (c.steps > movesLeft) ? HORIZON_PEN * (c.steps - movesLeft) * MOVE_COST : 0.0;
            c.eval = c.net + GAMMA * best2Net - horizonPen;
        }

        // Select best candidate
        if (!cands.isEmpty()) {
            cands.sort((a, b) -> {
                int cmp = Double.compare(b.eval, a.eval); // descending by evaluation
                if (cmp != 0) return cmp;
                // tie-breaker: fewer steps
                int sa = (a.steps < 0 ? Integer.MAX_VALUE : a.steps);
                int sb = (b.steps < 0 ? Integer.MAX_VALUE : b.steps);
                return Integer.compare(sa, sb);
            });
            
            Candidate best = cands.get(0);
            if (best.steps >= 0) lastBestPath = best.path; 
            else lastBestPath = new ArrayList<>();
            
            addPerceptOnce(String.format(java.util.Locale.ROOT,
                    "best_target(%s,%d,%d,%d,%.2f)",
                    best.color, best.next.x, best.next.y, best.steps, best.net));

            // Publish all candidates for agent decision-making
            for (Candidate c : cands) {
                if (c.steps >= 0) {
                    if (c.detourColor != null) {
                        addPerceptOnce(String.format(java.util.Locale.ROOT,
                                "candidate(%s,%d,%d,%d,%.2f,detour(%s,%.2f,%d))",
                                c.color, c.next.x, c.next.y, c.steps, c.net, c.detourColor, c.detourNet, c.detourSteps));
                    } else {
                        addPerceptOnce(String.format(java.util.Locale.ROOT,
                                "candidate(%s,%d,%d,%d,%.2f)",
                                c.color, c.next.x, c.next.y, c.steps, c.net));
                    }
                } else {
                    addPerceptOnce(String.format(java.util.Locale.ROOT,
                            "candidate(%s,%d,%d,%d,%s)",
                            c.color, goals.get(c.color).pos.x, goals.get(c.color).pos.y, -1, "-1.0e9"));
                }
            }
        } else {
            lastBestPath = new ArrayList<>();
        }
    }

    private void addPerceptOnce(String literal) { 
        addPercept(Literal.parseLiteral(literal)); 
    }
    private void addPerceptOnce(Literal lit) { 
        addPercept(lit); 
    }

    // ===== World Initialization =====

    /**
     * Place obstacles randomly on the grid
     */
    private void placeObstacles(int count) {
        obstacles.clear();
        while (obstacles.size() < count) {
            obstacles.add(new Cell(rng.nextInt(GRID_SIZE), rng.nextInt(GRID_SIZE)));
        }
    }

    /**
     * Place all goals at random positions
     */
    private void placeAllGoalsRandom() {
        goals.clear();
        placeGoal("green"); 
        placeGoal("yellow"); 
        placeGoal("blue"); 
        placeGoal("purple");
    }

    /**
     * Place a single goal of specified color
     */
    private void placeGoal(String color) { 
        goals.put(color, new Goal(color, randomFreeCell(), REWARD.get(color))); 
    }

    /**
     * Place agent at random free position
     */
    private void placeAgentRandom() { 
        agent = randomFreeCell(); 
    }

    /**
     * Find a random cell that is not occupied by obstacles, goals, or agent
     */
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

    /**
     * Check if a cell is occupied by obstacles, goals, or agent
     */
    private boolean isOccupied(Cell c) {
        if (obstacles.contains(c)) return true;
        for (Goal g : goals.values()) 
            if (g.pos.equals(c)) return true;
        return agent != null && agent.equals(c);
    }

    /**
     * Check if cell is within grid boundaries
     */
    private boolean isInside(Cell c) { 
        return c.x >= 0 && c.x < GRID_SIZE && c.y >= 0 && c.y < GRID_SIZE; 
    }

    // ===== A* Pathfinding Algorithm =====

    /**
     * Result of A* pathfinding containing reachability and path
     */
    private static class PathResult { 
        final boolean reachable; 
        final List<Cell> path; 
        
        PathResult(boolean r, List<Cell> p) {
            reachable = r; 
            path = p;
        } 
    }

    /**
     * Node for A* algorithm with g-score, f-score, and parent
     */
    private static class Node { 
        final Cell c; 
        final double g, f; 
        final Node parent; 
        
        Node(Cell c, double g, double f, Node p) {
            this.c = c; 
            this.g = g; 
            this.f = f; 
            this.parent = p;
        } 
    }

    /**
     * Candidate goal for agent evaluation
     */
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
        
        private Candidate(String color, Cell next, int steps, double net, List<Cell> path, String detourColor, double detourNet, int detourSteps) {
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
        
        static Candidate reachableWithDetour(String color, Cell next, int steps, double net, List<Cell> path, String detourColor, double detourNet, int detourSteps) { 
            return new Candidate(color, next, steps, net, path, detourColor, detourNet, detourSteps); 
        }
        
        static Candidate unreachable(Goal g) { 
            return new Candidate(g.color, g.pos, -1, -1.0e9, new ArrayList<>(), null, 0.0, 0); 
        }
    }

    /**
     * A* pathfinding algorithm with Manhattan distance heuristic
     */
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
                // Reconstruct path
                List<Cell> path = new ArrayList<>();
                Node n = current;
                while (n != null) { 
                    path.add(n.c); 
                    n = n.parent; 
                }
                Collections.reverse(path);
                return new PathResult(true, path);
            }
            closed.add(current.c);

            // Explore neighbors
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

    /**
     * Get all 4-connected neighbors of a cell
     */
    private List<Cell> neighbors(Cell c) {
        return Arrays.asList(
            new Cell(c.x + 1, c.y),
            new Cell(c.x - 1, c.y),
            new Cell(c.x, c.y + 1),
            new Cell(c.x, c.y - 1)
        );
    }

    /**
     * Manhattan distance heuristic for A*
     */
    private static int heuristic(Cell a, Cell b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }

    // ===== Online Detour Optimization =====

    /**
     * Information about a potential detour goal
     */
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

    /**
     * Find the best detour goal that is near the path and profitable
     */
    private DetourInfo findBestDetour(Cell start, Cell target, List<Cell> path, Map<String, Goal> goals) {
        DetourInfo bestDetour = null;
        double bestNetGain = 0.0;
        
        // Check each goal for potential detour
        for (Goal g : goals.values()) {
            if (g.pos.equals(target)) continue; // Skip target goal itself
            
            // Check if goal is "near" the path (within 2 steps)
            boolean isNearPath = false;
            for (Cell pathCell : path) {
                if (heuristic(pathCell, g.pos) <= 2) {
                    isNearPath = true;
                    break;
                }
            }
            
            if (!isNearPath) continue;
            
            // Calculate detour cost
            PathResult toDetour = aStar(start, g.pos);
            PathResult fromDetourToTarget = aStar(g.pos, target);
            
            if (!toDetour.reachable || !fromDetourToTarget.reachable) continue;
            
            int directSteps = path.size() - 1;
            int detourSteps = (toDetour.path.size() - 1) + (fromDetourToTarget.path.size() - 1);
            int extraSteps = detourSteps - directSteps;
            
            // Check profitability: net gain = reward - 0.01 * extraSteps > 0
            double netGain = g.reward - 0.01 * extraSteps;
            
            if (netGain > 0 && netGain > bestNetGain) {
                bestDetour = new DetourInfo(g.color, netGain, extraSteps, g.pos);
                bestNetGain = netGain;
            }
        }
        
        return bestDetour;
    }

    /**
     * Create a path that includes the detour goal
     */
    private List<Cell> createPathWithDetour(Cell start, Cell target, DetourInfo detour) {
        if (detour == null) {
            PathResult pr = aStar(start, target);
            return pr.reachable ? pr.path : new ArrayList<>();
        }
        
        // Create path: start -> detour goal -> target
        PathResult toDetour = aStar(start, detour.detourGoal);
        PathResult fromDetourToTarget = aStar(detour.detourGoal, target);
        
        if (!toDetour.reachable || !fromDetourToTarget.reachable) {
            // Fallback to direct path
            PathResult pr = aStar(start, target);
            return pr.reachable ? pr.path : new ArrayList<>();
        }
        
        List<Cell> combinedPath = new ArrayList<>(toDetour.path);
        // Remove duplicate detour goal from second path
        if (fromDetourToTarget.path.size() > 1) {
            combinedPath.addAll(fromDetourToTarget.path.subList(1, fromDetourToTarget.path.size()));
        }
        return combinedPath;
    }

    // ===== GUI Implementation =====

    /**
     * Initialize GUI components if not in headless mode
     */
    private void ensureGUI() {
        if (GraphicsEnvironment.isHeadless()) return;
        if (frame != null) return;
        
        SwingUtilities.invokeLater(() -> {
            frame = new JFrame("GoalEnv — 9×9 Grid (A* with Detours)");
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

    /**
     * Repaint GUI asynchronously
     */
    private void repaintAsync() {
        if (panel == null) return;
        SwingUtilities.invokeLater(() -> panel.repaint());
    }

    /**
     * Create snapshot of current world state for GUI rendering
     */
    private Snapshot snapshot() {
        Map<String, GoalCopy> gs = new HashMap<>();
        for (Map.Entry<String, Goal> e : goals.entrySet()) {
            Goal g = e.getValue();
            gs.put(e.getKey(), new GoalCopy(g.color, g.pos.x, g.pos.y, g.reward));
        }
        Set<Cell> obs = new HashSet<>(obstacles);
        List<Cell> path = new ArrayList<>(lastBestPath);
        return new Snapshot(GRID_SIZE,
                agent == null ? null : new Cell(agent.x, agent.y),
                obs, gs, time, MAX_MOVES - time, episode, totalReward, totalCost, (totalReward - totalCost), path);
    }

    /**
     * Immutable copy of goal for GUI rendering
     */
    private static class GoalCopy { 
        final String color; 
        final int x, y; 
        final double r; 
        
        GoalCopy(String c, int x, int y, double r) {
            this.color = c; 
            this.x = x; 
            this.y = y; 
            this.r = r;
        } 
    }

    /**
     * Immutable snapshot of world state for GUI rendering
     */
    private static class Snapshot {
        final int size; 
        final Cell agent; 
        final Set<Cell> obstacles; 
        final Map<String, GoalCopy> goals;
        final int time, movesLeft, episode; 
        final double totalR, totalC, score; 
        final List<Cell> bestPath;
        
        Snapshot(int size, Cell agent, Set<Cell> obstacles, Map<String, GoalCopy> goals,
                 int time, int movesLeft, int episode, double totalR, double totalC, double score, List<Cell> bestPath) {
            this.size = size; 
            this.agent = agent; 
            this.obstacles = obstacles; 
            this.goals = goals;
            this.time = time; 
            this.movesLeft = movesLeft; 
            this.episode = episode;
            this.totalR = totalR; 
            this.totalC = totalC; 
            this.score = score; 
            this.bestPath = bestPath;
        }
    }

    /**
     * Main grid panel for rendering the world
     */
    private class GridPanel extends JPanel {
        GridPanel() {
            setPreferredSize(new Dimension(PADDING*2 + CELL*GRID_SIZE, PADDING*2 + CELL*GRID_SIZE + 100));
            setBackground(Color.white);
        }
        
        @Override 
        protected void paintComponent(Graphics g0) {
            super.paintComponent(g0);
            Graphics2D g = (Graphics2D) g0;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            Snapshot s;
            synchronized (GoalEnv.this) { s = snapshot(); }

            // Draw grid background
            g.setColor(new Color(248,248,248));
            g.fillRoundRect(PADDING-6, PADDING-6, CELL*s.size+12, CELL*s.size+12, 12,12);

            // Draw grid lines
            g.setColor(new Color(210,210,210));
            for (int i=0;i<=s.size;i++) {
                int x = PADDING + i*CELL;
                int y = PADDING + i*CELL;
                g.drawLine(PADDING, y, PADDING + CELL*s.size, y);
                g.drawLine(x, PADDING, x, PADDING + CELL*s.size);
            }

            // Draw obstacles
            g.setColor(Color.black);
            for (Cell o : s.obstacles) {
                int x = PADDING + o.x*CELL;
                int y = PADDING + o.y*CELL;
                g.fillRoundRect(x+4, y+4, CELL-8, CELL-8, 10,10);
            }

            // Draw best path (dashed line)
            if (s.bestPath != null && s.bestPath.size() > 1) {
                java.awt.Stroke old = g.getStroke();
                float[] dash = {6f,6f};
                g.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 10f, dash, 0f));
                g.setColor(new Color(80,80,200));
                for (int i=0;i<s.bestPath.size()-1;i++) {
                    Cell a = s.bestPath.get(i), b = s.bestPath.get(i+1);
                    int ax = PADDING + a.x*CELL + CELL/2;
                    int ay = PADDING + a.y*CELL + CELL/2;
                    int bx = PADDING + b.x*CELL + CELL/2;
                    int by = PADDING + b.y*CELL + CELL/2;
                    g.drawLine(ax, ay, bx, by);
                }
                g.setStroke(old);
            }

            // Draw goals
            for (GoalCopy gc : s.goals.values()) {
                Color col = goalColor(gc.color);
                g.setColor(col);
                int x = PADDING + gc.x*CELL;
                int y = PADDING + gc.y*CELL;
                int d = CELL-12;
                java.awt.Shape oval = new Ellipse2D.Double(x+6, y+6, d, d);
                g.fill(oval);
                g.setColor(new Color(0,0,0,120));
                g.draw(oval);
                // Draw reward label
                g.setFont(g.getFont().deriveFont(Font.BOLD, 12f));
                String lab = gc.color + "=" + format2(gc.r);
                drawCenteredString(g, lab, new Rectangle(x, y, CELL, CELL), g.getFont());
            }

            // Draw agent
            if (s.agent != null) {
                int x = PADDING + s.agent.x*CELL;
                int y = PADDING + s.agent.y*CELL;
                int d = CELL-18;
                g.setColor(new Color(220,60,50));
                g.fill(new Ellipse2D.Double(x+9, y+9, d, d));
                g.setColor(Color.white);
                g.setStroke(new BasicStroke(2f));
                g.drawOval(x+9, y+9, d, d);
            }

            // Draw HUD
            int hudY = PADDING*2 + CELL*s.size + 12;
            g.setColor(new Color(30,30,30));
            g.setFont(g.getFont().deriveFont(Font.PLAIN, 14f));
            String hud = String.format(java.util.Locale.ROOT,
                    "Episode %d  |  t=%d  moves_left=%d  |  reward=%.2f  cost=%.2f  score=%.2f",
                    s.episode, s.time, s.movesLeft, s.totalR, s.totalC, s.score);
            g.drawString(hud, PADDING, hudY);

            // Draw legend
            g.setFont(g.getFont().deriveFont(Font.PLAIN, 12f));
            g.drawString("Legend: Agent (red), Goals (colored), Obstacles (black), Best path (dashed)", PADDING, hudY + 18);
        }

        /**
         * Draw centered text in a rectangle
         */
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

    /**
     * Get color for goal based on name
     */
    private static Color goalColor(String name) {
        if ("green".equals(name))  return new Color(40, 170, 80);
        if ("yellow".equals(name)) return new Color(220, 190, 40);
        if ("blue".equals(name))   return new Color(70, 120, 220);
        if ("purple".equals(name)) return new Color(150, 70, 190);
        return new Color(100, 100, 100);
    }

    // ===== Episode Logging =====

    /**
     * Log episode results to text file
     */
    private void logEpisodeResult(int episodeNumber, double reward, double cost, double score, int steps) {
        try (FileWriter writer = new FileWriter(logFileName, true)) {
            LocalDateTime now = LocalDateTime.now();
            String timestamp = now.format(DateTimeFormatter.ofPattern("yyyy-MM-dd HH:mm:ss"));
            
            writer.write(String.format(java.util.Locale.ROOT, 
                "Episode %d | %s | Reward: %.2f | Cost: %.2f | Score: %.2f | Steps: %d\n",
                episodeNumber, timestamp, reward, cost, score, steps));
            writer.flush();
        } catch (IOException e) {
            System.err.println("Failed to write episode log: " + e.getMessage());
        }
    }

    /**
     * Initialize log file with headers
     */
    private void initializeLogFile() {
        try (FileWriter writer = new FileWriter(logFileName, false)) {
            writer.write("=== GoalEnv Episode Rewards Log ===\n");
            writer.write("Format: Episode | Timestamp | Reward | Cost | Score | Steps\n");
            writer.write("=====================================\n");
            writer.flush();
        } catch (IOException e) {
            System.err.println("Failed to initialize log file: " + e.getMessage());
        }
    }

    // ===== Utility Methods =====

    /**
     * Parse command line arguments for configuration
     */
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
            } catch (NumberFormatException ignored) { }
        }
    }

    /**
     * Remove quotes from string literals
     */
    private static String stripQuotes(String s) {
        if (s == null) return null;
        int n = s.length();
        if (n >= 2 && ((s.charAt(0) == '\'' && s.charAt(n-1) == '\'') || (s.charAt(0) == '"' && s.charAt(n-1) == '"'))) {
            return s.substring(1, n-1);
        }
        return s;
    }

    /**
     * Format double to 2 decimal places
     */
    private static String format2(double v) {
        return String.format(java.util.Locale.ROOT, "%.2f", v);
    }

    /**
     * Control panel with speed slider
     */
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
}