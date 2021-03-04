package algorithms;

import map.Cell;
import map.Map;
import map.MapConstants;
import robot.Robot;
import robot.RobotConstants;
import robot.RobotConstants.DIRECTION;
import robot.RobotConstants.MOVEMENT;
import utils.CommMgr;
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
    private int lastCalibrate;
    private boolean calibrationMode;

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
        if (bot.getRealBot()) {
            System.out.println("Starting calibration...");

            //CommMgr.getCommMgr().recvMsg();

//            if (bot.getRealBot()) {
//                bot.move(MOVEMENT.LEFT, false);
//                //CommMgr.getCommMgr().recvMsg();
//                bot.move(MOVEMENT.CALIBRATE, false);
//                //CommMgr.getCommMgr().recvMsg();
//                bot.move(MOVEMENT.LEFT, false);
//                //CommMgr.getCommMgr().recvMsg();
//                bot.move(MOVEMENT.CALIBRATE, false);
//                //CommMgr.getCommMgr().recvMsg();
//                bot.move(MOVEMENT.RIGHT, false);
//                //CommMgr.getCommMgr().recvMsg();
//                bot.move(MOVEMENT.CALIBRATE, false);
//                //CommMgr.getCommMgr().recvMsg();
//                bot.move(MOVEMENT.RIGHT, false);
//            }

            while (true) {
                System.out.println("Waiting for to Start...");
                String msg = CommMgr.getCommMgr().recvMsg();
                if(!msg.isEmpty()){
                    break;
                }
            }
        }

        System.out.println("\nExploring...");

        startTime = System.currentTimeMillis();
        endTime = startTime + (timeLimit * 1000);

        if (bot.getRealBot()) {
            String start = "start";
            CommMgr.getCommMgr().sendMsg(start, CommMgr.AR);
        }
        senseAndRepaint();

        areaExplored = calculateAreaExplored();
        // System.out.println("Explored Area: " + areaExplored);

        explorationLoop(bot.getRobotPosRow(), bot.getRobotPosCol());
    }

    /**
     * Loops through robot movements until one (or more) of the following conditions is met:
     * 1. Robot is back at (r, c)
     * 2. areaExplored > coverageLimit
     * 3. System.currentTimeMillis() > endTime
     */
    private void explorationLoop(int r, int c) {
        // System.out.println("explorationLoop activated");
        do {
            nextMove();

            areaExplored = calculateAreaExplored();
            // System.out.println("Area explored: " + areaExplored);

            //System.out.println("This is the end of one move\n");

            // This is the stopping condition where r and c are the robot's starting positions
            if (bot.getRobotPosRow() == r && bot.getRobotPosCol() == c) {
                if (areaExplored >= 100) {
                    break;
                }
            }
        } while (areaExplored <= coverageLimit && System.currentTimeMillis() <= endTime);

        //goHome();
    }

    /**
     * Determines the next move for the robot and executes it accordingly.
     */
    private void nextMove() {
        //System.out.println("nextMove activated");

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
     * Returns the robot to START after exploration and points the bot northwards.
     */
    /*private void goHome() {
        // System.out.println("Exploration complete!");

        if (!bot.getTouchedGoal() && coverageLimit == 300 && timeLimit == 3600) {
            FastestPathAlgo goToGoal = new FastestPathAlgo(exploredMap, bot, realMap);
            goToGoal.runFastestPath(RobotConstants.GOAL_ROW, RobotConstants.GOAL_COL);
        }

        if (bot.getRobotPosRow()!=RobotConstants.START_ROW || bot.getRobotPosCol()!=RobotConstants.START_COL){
            FastestPathAlgo returnToStart = new FastestPathAlgo(exploredMap, bot, realMap);
            returnToStart.runFastestPath(RobotConstants.START_ROW, RobotConstants.START_COL);
            System.out.println();
        }

        areaExplored = calculateAreaExplored();
        System.out.printf("%.2f%% Coverage\n", (areaExplored / 300.0) * 100.0);
        // System.out.println(", " + areaExplored + " Cells");
        // System.out.println((System.currentTimeMillis() - startTime) / 1000 + " Seconds");

        if (bot.getRealBot()) {
            turnBotDirection(DIRECTION.WEST);
            moveBot(MOVEMENT.CALIBRATE);
            turnBotDirection(DIRECTION.SOUTH);
            moveBot(MOVEMENT.CALIBRATE);
            turnBotDirection(DIRECTION.WEST);
            moveBot(MOVEMENT.CALIBRATE);
        }
        turnBotDirection(DIRECTION.NORTH);
    }*/

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
        //System.out.println("moveBot (Algo) Activated");

        // Insert pledge algorithm here
        /*
        General flow:
        - If an obstacle is detected to the robot's left/right, begin pledge algorithm
        - This will be inside a while loop (or maybe not) and will continue until the very starting waypoint is reached again
        - The starting waypoint should not be visited and this should be the main condition to start the pledge algorithm
         */

        /*if (bot.getRobotPosCol() != 1 & bot.getRobotPosRow() != 1) {
            int currentColumn = 0;
            int currentRow = 0;
            DIRECTION currentDirection = null;

            int startColumn = bot.getRobotPosCol();
            int startRow = bot.getRobotPosRow();

            DIRECTION startDirection = bot.getRobotCurDir();

            int[] current_SensorData = bot.sense(exploredMap, realMap);

            if (current_SensorData[0] == 1){
                boolean flag = false;

                System.out.println("Entering Pledge");

                while (flag == false) {

                    int[] sensorData = bot.sense(exploredMap, realMap);
                    currentColumn = bot.getRobotPosCol();
                    currentRow = bot.getRobotPosRow();
                    currentDirection = bot.getRobotCurDir();

                    if (sensorData[0] == -1 & lookLeft()) {
                        bot.move(MOVEMENT.FORCEDLEFT);
                    } else if (sensorData[0] > -1 & lookForward()) {
                        bot.move(MOVEMENT.FORWARD);
                    } else if (sensorData[0] > -1 & lookRight() & !lookForward()) {
                        bot.move(MOVEMENT.FORCEDRIGHT);
                    } else {
                        System.out.println("Exited the Pledge");
                        break;
                    }

                    *//*if (currentColumn == startColumn & currentRow == startRow & currentDirection == startDirection) {
                        flag = true;
                        System.out.println("Exited the Pledge");
                    }*//*
                }
                System.out.println("Exited the Pledge");
            }
        }*/

        bot.move(m);
        exploredMap.repaint();

        if (m != MOVEMENT.CALIBRATE) {
            senseAndRepaint();
        } else {
            CommMgr commMgr = CommMgr.getCommMgr();
            commMgr.recvMsg();
        }

        if (bot.getRealBot() && !calibrationMode) {
            calibrationMode = true;

            if (canCalibrateOnTheSpot(bot.getRobotCurDir())) {
                lastCalibrate = 0;
                moveBot(MOVEMENT.CALIBRATE);
            } else {
                lastCalibrate++;
                if (lastCalibrate >= 5) {
                    DIRECTION targetDir = getCalibrationDirection();
                    if (targetDir != null) {
                        lastCalibrate = 0;
                        calibrateBot(targetDir);
                    }
                }
            }
            calibrationMode = false;
        }
    }

    /**
     * Sets the bot's sensors, processes the sensor data and repaints the map.
     */
    private void senseAndRepaint() {
        //System.out.println("senseAndRepaint Activated");
        bot.setSensors();
        bot.sense(exploredMap, realMap);
        exploredMap.repaint();
    }

    /**
     * Checks if the robot can calibrate at its current position given a direction.
     */
    private boolean canCalibrateOnTheSpot(DIRECTION botDir) {
        //System.out.println("canCalibrateOnTheSpot Activated");
        int row = bot.getRobotPosRow();
        int col = bot.getRobotPosCol();

        switch (botDir) {
            case NORTH:
                return exploredMap.getIsObstacleOrWall(row + 2, col - 1) && exploredMap.getIsObstacleOrWall(row + 2, col) && exploredMap.getIsObstacleOrWall(row + 2, col + 1);
            case EAST:
                return exploredMap.getIsObstacleOrWall(row + 1, col + 2) && exploredMap.getIsObstacleOrWall(row, col + 2) && exploredMap.getIsObstacleOrWall(row - 1, col + 2);
            case SOUTH:
                return exploredMap.getIsObstacleOrWall(row - 2, col - 1) && exploredMap.getIsObstacleOrWall(row - 2, col) && exploredMap.getIsObstacleOrWall(row - 2, col + 1);
            case WEST:
                return exploredMap.getIsObstacleOrWall(row + 1, col - 2) && exploredMap.getIsObstacleOrWall(row, col - 2) && exploredMap.getIsObstacleOrWall(row - 1, col - 2);
        }
        return false;
    }

    /**
     * Returns a possible direction for robot calibration or null, otherwise.
     */
    private DIRECTION getCalibrationDirection() {
        //System.out.println("getCalibrationDirection Activated");
        DIRECTION origDir = bot.getRobotCurDir();
        DIRECTION dirToCheck;

        dirToCheck = DIRECTION.getNext(origDir);                    // right turn
        if (canCalibrateOnTheSpot(dirToCheck)) return dirToCheck;

        dirToCheck = DIRECTION.getPrevious(origDir);                // left turn
        if (canCalibrateOnTheSpot(dirToCheck)) return dirToCheck;

        dirToCheck = DIRECTION.getPrevious(dirToCheck);             // u turn
        if (canCalibrateOnTheSpot(dirToCheck)) return dirToCheck;

        return null;
    }

    /**
     * Turns the bot in the needed direction and sends the CALIBRATE movement. Once calibrated, the bot is turned back
     * to its original direction.
     */
    private void calibrateBot(DIRECTION targetDir) {
        //System.out.println("calibrateBot Activated");
        DIRECTION origDir = bot.getRobotCurDir();

        turnBotDirection(targetDir);
        moveBot(MOVEMENT.CALIBRATE);
        turnBotDirection(origDir);
    }

    /**
     * Turns the robot to the required direction.
     */
    private void turnBotDirection(DIRECTION targetDir) {
        //System.out.println("turnBotDirection Activated");
        int numOfTurn = Math.abs(bot.getRobotCurDir().ordinal() - targetDir.ordinal());
        if (numOfTurn > 2) numOfTurn = numOfTurn % 2;

        if (numOfTurn == 1) {
            if (DIRECTION.getNext(bot.getRobotCurDir()) == targetDir) {
                moveBot(MOVEMENT.RIGHT);
            } else {
                moveBot(MOVEMENT.LEFT);
            }
        } else if (numOfTurn == 2) {
            moveBot(MOVEMENT.RIGHT);
            moveBot(MOVEMENT.RIGHT);
        }
    }
}
