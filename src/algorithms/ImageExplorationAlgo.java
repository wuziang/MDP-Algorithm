package algorithms;

import map.Cell;
import map.Map;
import map.MapConstants;
import robot.Robot;
import robot.RobotConstants;
import robot.RobotConstants.DIRECTION;
import robot.RobotConstants.MOVEMENT;
import utils.CommMgr;

import java.util.Arrays;

/**
 * Exploration algorithm for the robot.
 */

public class ImageExplorationAlgo {
    private final Map exploredMap;
    private final Map realMap;
    private final Robot bot;

    private final int coverageLimit;
    private final int timeLimit;
    private int areaExplored;

    private long startTime;
    private long endTime;

    private int[] sensorData;

    public ImageExplorationAlgo(Map exploredMap, Map realMap, Robot bot, int coverageLimit, int timeLimit) {
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
        System.out.println("\nFinding Image...");

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
            // This will determine the robot's next move to make
            nextMove();
            areaExplored = calculateAreaExplored();

            currentPosition();

            // This is the stopping condition where r and c are the robot's starting positions
            if (bot.getRobotPosRow() == r && bot.getRobotPosCol() == c) {
                if (areaExplored >= 100) {
                    break;
                }
            }
        } while (areaExplored <= coverageLimit && System.currentTimeMillis() <= endTime);

        // goHome();
    }

    /**
     * Determines the next move for the robot and executes it accordingly.
     */
    private void nextMove() {
        if (lookRight()) {
            moveBot(MOVEMENT.RIGHT);
            if (lookForward()) moveBot(MOVEMENT.FORWARD);
        } else if (lookForward()) {
            moveBot(MOVEMENT.FORWARD);
        } else if (lookLeft()) {
            moveBot(MOVEMENT.LEFT);
            if (lookForward()) moveBot(MOVEMENT.FORWARD);
        } else {
            moveBot(MOVEMENT.RIGHT);
            moveBot(MOVEMENT.RIGHT);
        }
    }

    /**
     * Checks whether there are obstacles to turn towards for image finding
     */
    public void currentPosition(){
        DIRECTION originalDirection = bot.getRobotCurDir();

        if (northBlocked()){
            turnBotDirection(DIRECTION.NORTH);

            // Send message to Rpi to take photo
            if(bot.getRealBot()) {
                bot.takePhoto();
            }
            // Return back to original direction
            turnBotDirection(originalDirection);
        }

        if (eastBlocked()){
            turnBotDirection(DIRECTION.EAST);

            // Send message to Rpi to take photo
            if(bot.getRealBot()) {
                bot.takePhoto();
            }
            // Return back to original direction
            turnBotDirection(originalDirection);
        }

        if (southBlocked()){
            turnBotDirection(DIRECTION.SOUTH);

            // Send message to Rpi to take photo
            if(bot.getRealBot()) {
                bot.takePhoto();
            }
            // Return back to original direction
            turnBotDirection(originalDirection);
        }

        if (westBlocked()){
            turnBotDirection(DIRECTION.WEST);

            // Send message to Rpi to take photo
            if(bot.getRealBot()) {
                bot.takePhoto();
            }
            // Return back to original direction
            turnBotDirection(originalDirection);
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
     * Returns true if the North cell has an obstacle
     */
    private boolean northBlocked(){
        DIRECTION currentDirection = bot.getRobotCurDir();

        // We need to consider the possibility the robot is along the wall (so we can ignore the wall as an obstacle)
        int currentRow = bot.getRobotPosRow();
        int currentColumn = bot.getRobotPosCol();

        // We consider different scenarios where the North side may be blocked
        if (sensorData[2]==1 & currentDirection==DIRECTION.NORTH & currentRow != 18){
            return true;
        }
        if (sensorData[0]==1 & currentDirection==DIRECTION.EAST & currentRow != 18){
            return true;
        }
        if (sensorData[4]==1 & currentDirection==DIRECTION.WEST & currentRow != 18){
            return true;
        }
        return false;
    }

    /**
     * Returns true if the East cell has an obstacle
     */
    private boolean eastBlocked(){
        DIRECTION currentDirection = bot.getRobotCurDir();

        // We need to consider the possibility the robot is along the wall (so we can ignore the wall as an obstacle)
        int currentRow = bot.getRobotPosRow();
        int currentColumn = bot.getRobotPosCol();

        // We consider different scenarios where the East side may be blocked
        if (sensorData[4]==1 & currentDirection==DIRECTION.NORTH & currentColumn != 13){
            return true;
        }
        if (sensorData[0]==1 & currentDirection==DIRECTION.SOUTH & currentColumn != 13){
            return true;
        }
        return false;
    }

    /**
     * Returns true if the South cell has an obstacle
     */
    private boolean southBlocked(){
        DIRECTION currentDirection = bot.getRobotCurDir();

        // We need to consider the possibility the robot is along the wall (so we can ignore the wall as an obstacle)
        int currentRow = bot.getRobotPosRow();
        int currentColumn = bot.getRobotPosCol();

        // We consider different scenarios where the South side may be blocked
        if (sensorData[2]==1 & currentDirection==DIRECTION.SOUTH & currentRow != 1){
            return true;
        }
        if (sensorData[4]==1 & currentDirection==DIRECTION.EAST & currentRow != 1){
            return true;
        }
        if (sensorData[0]==1 & currentDirection==DIRECTION.WEST & currentRow != 1){
            return true;
        }
        return false;
    }

    /**
     * Returns true if the West cell has an obstacle
     */
    private boolean westBlocked(){
        DIRECTION currentDirection = bot.getRobotCurDir();

        // We need to consider the possibility the robot is along the wall (so we can ignore the wall as an obstacle)
        int currentRow = bot.getRobotPosRow();
        int currentColumn = bot.getRobotPosCol();

        // We consider different scenarios where the West side may be blocked
        if (sensorData[0]==1 & currentDirection==DIRECTION.NORTH & currentColumn != 1){
            return true;
        }
        if (sensorData[4]==1 & currentDirection==DIRECTION.SOUTH & currentColumn != 1){
            return true;
        }
        return false;
    }

    /**
     * Returns the robot to START after exploration and points the bot northwards.
     */
    private void goHome() {
        FastestPathAlgo returnToStart = new FastestPathAlgo(exploredMap, bot);
        String output = returnToStart.runFastestPath(RobotConstants.START_ROW, RobotConstants.START_COL);

        areaExplored = calculateAreaExplored();
        System.out.printf("%.2f%% Coverage", (areaExplored / 300.0) * 100.0);
    }

    /**
     * Returns true for cells that are explored and not obstacles.
     */
    private boolean isExploredNotObstacle(int r, int c) {
        if (exploredMap.checkValidCoordinates(r, c)) {
            Cell tmp = exploredMap.getCell(r, c);
            return (tmp.getIsExplored() && !tmp.getIsObstacle());
        }
        return false;
    }

    /**
     * Returns true for cells that are explored and are obstacles.
     */
    private boolean isExploredObstacle(int r, int c) {
        if (exploredMap.checkValidCoordinates(r, c)) {
            Cell tmp = exploredMap.getCell(r, c);
            return (tmp.getIsExplored() && tmp.getIsObstacle());
        }
        return false;
    }

    /**
     * Returns true for cells that are explored, not virtual walls and not obstacles.
     */
    private boolean isExploredAndFree(int r, int c) {
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
        sensorData = bot.sense(exploredMap, realMap);
    }

    /**
     * Turns the robot to the required direction.
     */
    private void turnBotDirection(DIRECTION targetDir) {
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
