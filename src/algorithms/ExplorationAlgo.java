package algorithms;

import map.Cell;
import map.Map;
import map.MapConstants;
import robot.Robot;
import robot.RobotConstants;
import robot.RobotConstants.DIRECTION;
import robot.RobotConstants.MOVEMENT;
import utils.CommMgr;
import utils.MapDescriptor;

/**
 * Exploration algorithm for the robot.
 */

public class ExplorationAlgo {
    private final Map exploredMap;
    private final Map realMap;
    private final Robot bot;

    private final int coverageLimit;
    private final int timeLimit;

    private int areaExplored;

    private long startTime;
    private long endTime;

    public ExplorationAlgo(Map exploredMap, Map realMap, Robot bot, int coverageLimit, int timeLimit) {
        this.exploredMap = exploredMap;
        this.realMap = realMap;
        this.bot = bot;
        this.coverageLimit = coverageLimit;
        this.timeLimit = timeLimit;
    }

    /**
     * Main method that is called to start the exploration.
     */
    public void runExploration() {
        System.out.println("\nExploring...");

        if (bot.getRealBot()) {
            while (true) {
                String msg = CommMgr.getCommMgr().recvMsg();
                if(!msg.isEmpty()){
                    break;
                }
            }
        }

        startTime = System.currentTimeMillis();
        endTime = startTime + (timeLimit * 1000);

        senseAndRepaint();

        areaExplored = calculateAreaExplored();
        explorationLoop(bot.getRobotPosRow(), bot.getRobotPosCol());
    }

    /**
     * Loops through robot movements until one (or more) of the following conditions is met:
     * 1. Robot is back at (r, c)
     * 2. areaExplored > coverageLimit
     * 3. System.currentTimeMillis() > endTime
     */
    private void explorationLoop(int r, int c) {
        do {
            nextMove();
            areaExplored = calculateAreaExplored();

            // This is the stopping condition where r and c are the robot's starting positions
            if (bot.getRobotPosRow() == r && bot.getRobotPosCol() == c) {
                if (areaExplored >= 100) {
                    break;
                }
            }
        } while (areaExplored <= coverageLimit && System.currentTimeMillis() <= endTime);

        areaExplored = calculateAreaExplored();
        System.out.printf("\nExploration Coverage %.2f%%\n", (areaExplored / 300.0) * 100.0);
        String[] mapStrings = MapDescriptor.generateMapDescriptor(exploredMap);
        System.out.println("P1: " + mapStrings[0]);
        System.out.println("P2: " + mapStrings[1]);
    }

    /**
     * Determines the next move for the robot and executes it accordingly.
     */
    private void nextMove() {
        if (lookRight()) {
            moveBot(MOVEMENT.RIGHT);
            if (lookForward())
                moveBot(MOVEMENT.FORWARD);
        } else if (lookForward()) {
            moveBot(MOVEMENT.FORWARD);
        } else if (lookLeft()) {
            moveBot(MOVEMENT.LEFT);
            if (lookForward())
                moveBot(MOVEMENT.FORWARD);
        } else {
            moveBot(MOVEMENT.RIGHT);
            moveBot(MOVEMENT.RIGHT);
        }
    }

    /**
     * Returns true if the right side of the robot is free to move into.
     */
    private boolean lookRight() {
        switch (bot.getRobotCurDir()) {
            case NORTH:
                return eastFree();
            case EAST:
                return southFree();
            case SOUTH:
                return westFree();
            case WEST:
                return northFree();
        }
        return false;
    }

    /**
     * Returns true if the robot is free to move forward.
     */
    private boolean lookForward() {
        switch (bot.getRobotCurDir()) {
            case NORTH:
                return northFree();
            case EAST:
                return eastFree();
            case SOUTH:
                return southFree();
            case WEST:
                return westFree();
        }
        return false;
    }

    /**
     * * Returns true if the left side of the robot is free to move into.
     */
    private boolean lookLeft() {
        switch (bot.getRobotCurDir()) {
            case NORTH:
                return westFree();
            case EAST:
                return northFree();
            case SOUTH:
                return eastFree();
            case WEST:
                return southFree();
        }
        return false;
    }

    /**
     * Returns true if the robot can move to the north cell.
     */
    private boolean northFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow + 1, botCol - 1) && isExploredAndFree(botRow + 1, botCol) && isExploredNotObstacle(botRow + 1, botCol + 1));
    }

    /**
     * Returns true if the robot can move to the east cell.
     */
    private boolean eastFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow - 1, botCol + 1) && isExploredAndFree(botRow, botCol + 1) && isExploredNotObstacle(botRow + 1, botCol + 1));
    }

    /**
     * Returns true if the robot can move to the south cell.
     */
    private boolean southFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow - 1, botCol - 1) && isExploredAndFree(botRow - 1, botCol) && isExploredNotObstacle(botRow - 1, botCol + 1));
    }

    /**
     * Returns true if the robot can move to the west cell.
     */
    private boolean westFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow - 1, botCol - 1) && isExploredAndFree(botRow, botCol - 1) && isExploredNotObstacle(botRow + 1, botCol - 1));
    }

    /**
     * Returns true for cells that are explored and not obstacles.
     */
    private boolean isExploredNotObstacle(int r, int c) {
        //System.out.println("isExploredNotObstacle Checked");
        if (exploredMap.checkValidCoordinates(r, c)) {
            Cell tmp = exploredMap.getCell(r, c);
            return (tmp.getIsExplored() && !tmp.getIsObstacle());
        }
        return false;
    }

    /**
     * Returns true for cells that are explored, not virtual walls and not obstacles.
     */
    private boolean isExploredAndFree(int r, int c) {
        //System.out.println("isExploredAndFree Checked");
        if (exploredMap.checkValidCoordinates(r, c)) {
            Cell b = exploredMap.getCell(r, c);
            return (b.getIsExplored() && !b.getIsVirtualWall() && !b.getIsObstacle());
        }
        return false;
    }

    /**
     * Returns the number of cells explored in the grid.
     */
    private int calculateAreaExplored() {
        int result = 0;
        for (int r = 0; r < MapConstants.MAP_ROWS; r++) {
            for (int c = 0; c < MapConstants.MAP_COLS; c++) {
                if (exploredMap.getCell(r, c).getIsExplored()) {
                    result++;
                }
            }
        }
        return result;
    }

    /**
     * Moves the bot, repaints the map and calls senseAndRepaint().
     */
    private void moveBot(MOVEMENT m) {
        bot.move(m);
        senseAndRepaint();
    }

    /**
     * Sets the bot's sensors, processes the sensor data and repaints the map.
     */
    private void senseAndRepaint() {
        exploredMap.repaint();

        bot.setSensors();
        bot.sense(exploredMap, realMap);
    }
}
