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

public class ImageProcessingAlgo extends ExplorationAlgo {
    private final Map exploredMap;
    private final Map realMap;
    private final Robot bot;

    private final int timeLimit;

    private long startTime;
    private long endTime;

    private int lastCalibrate;
    private boolean calibrationMode;

    private int[] sensorData;
    private int foundImage=0;
    private int takenImage=0;

    public ImageProcessingAlgo(Map exploredMap, Map realMap, Robot bot, int coverageLimit, int timeLimit) {
        super(exploredMap, realMap, bot, coverageLimit, timeLimit);
        this.exploredMap = exploredMap;
        this.realMap = realMap;
        this.bot = bot;
        this.timeLimit = timeLimit;
    }

    /**
     * Main method that is called to start the exploration.
     */
    public void runImage() {
        System.out.println("\nProcessing Image...");

        if (bot.getRealBot()) {
            // TODO: Start from Android
            CommMgr.getCommMgr().recvMsg();

            bot.move(MOVEMENT.LEFT);
            bot.move(MOVEMENT.CALIBRATE);
            bot.move(MOVEMENT.LEFT);
            bot.move(MOVEMENT.CALIBRATE);
            bot.move(MOVEMENT.RIGHT);
            bot.move(MOVEMENT.CALIBRATE);
            bot.move(MOVEMENT.RIGHT);
        }

        startTime = System.currentTimeMillis();
        endTime = startTime + (timeLimit * 1000);

        senseAndRepaint();
        imageLoop(bot.getRobotPosRow(), bot.getRobotPosCol());
    }

    /**
     * Loops through robot movements until one (or more) of the following conditions is met:
     * 1. Robot is back at (r, c)
     * 2. foundImage == 6
     * 3. System.currentTimeMillis() > endTime
     */
    private void imageLoop(int r, int c) {
        do {
            // This will determine the robot's next move to make
            nextMove();

            northProcess();
            eastProcess();
            southProcess();
            westProcess();

            if (bot.getRobotPosRow() == r && bot.getRobotPosCol() == c) {
                break;
            }
        } while (foundImage<6 && System.currentTimeMillis() <= endTime);

        double sides = calculateSidesPossible();
        System.out.printf("\nImage Processing Coverage %.2f%%", (takenImage/sides) * 100.0);

        if(bot.getRealBot()) {
            System.out.printf("\nImages Found: %d\n", foundImage);
        }
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
    private void northProcess(){
        DIRECTION currentDirection = bot.getRobotCurDir();

        // We need to consider the possibility the robot is along the wall (so we can ignore the wall as an obstacle)
        int currentRow = bot.getRobotPosRow();
        int currentColumn = bot.getRobotPosCol();

        int distance = -1;
        // We consider different scenarios where the North side may be blocked
        if (sensorData[2]!=-1 & currentDirection==DIRECTION.NORTH){
            distance = sensorData[2];
        }
        if (sensorData[0]!=-1 & currentDirection==DIRECTION.EAST){
            distance = sensorData[0];
        }
        if (sensorData[4]!=-1 & currentDirection==DIRECTION.WEST){
            distance = sensorData[4];
        }

        if(distance==-1 || distance>RobotConstants.CAMERA_RANGE) return;

        int cellRow = currentRow+1+distance;
        if(cellRow>=MapConstants.MAP_ROWS) return;

        int index = DIRECTION.SOUTH.getIndex();
        if(exploredMap.getCell(cellRow, currentColumn).getIsProcessed(index)) return;

        exploredMap.getCell(cellRow, currentColumn).setIsProcessed(index, true);

        turnCameraDirection(DIRECTION.NORTH);

        if(takePhoto(cellRow, currentColumn, DIRECTION.SOUTH.toString())) foundImage++;;
        takenImage++;

        turnBotDirection(currentDirection);
    }

    /**
     * Returns true if the East cell has an obstacle
     */
    private void eastProcess(){
        DIRECTION currentDirection = bot.getRobotCurDir();

        // We need to consider the possibility the robot is along the wall (so we can ignore the wall as an obstacle)
        int currentRow = bot.getRobotPosRow();
        int currentColumn = bot.getRobotPosCol();

        int distance = -1;
        // We consider different scenarios where the East side may be blocked
        if (sensorData[2]!=-1 & currentDirection==DIRECTION.EAST){
            distance = sensorData[2];
        }
        if (sensorData[4]!=-1 & currentDirection==DIRECTION.NORTH){
            distance = sensorData[4];
        }
        if (sensorData[0]!=-1 & currentDirection==DIRECTION.SOUTH){
            distance = sensorData[0];
        }

        if(distance==-1 || distance>RobotConstants.CAMERA_RANGE) return;

        int cellColumn = currentColumn+1+distance;
        if(cellColumn>=MapConstants.MAP_COLS) return;

        int index = DIRECTION.WEST.getIndex();
        if(exploredMap.getCell(currentRow, cellColumn).getIsProcessed(index)) return;

        exploredMap.getCell(currentRow, cellColumn).setIsProcessed(index, true);

        turnCameraDirection(DIRECTION.EAST);

        if(takePhoto(currentRow, cellColumn, DIRECTION.WEST.toString())) foundImage++;
        takenImage++;

        turnBotDirection(currentDirection);
    }

    /**
     * Returns true if the South cell has an obstacle
     */
    private void southProcess(){
        DIRECTION currentDirection = bot.getRobotCurDir();

        // We need to consider the possibility the robot is along the wall (so we can ignore the wall as an obstacle)
        int currentRow = bot.getRobotPosRow();
        int currentColumn = bot.getRobotPosCol();

        int distance = -1;
        // We consider different scenarios where the South side may be blocked
        if (sensorData[2]!=-1 & currentDirection==DIRECTION.SOUTH){
            distance = sensorData[2];
        }
        if (sensorData[4]!=-1 & currentDirection==DIRECTION.EAST){
            distance = sensorData[4];
        }
        if (sensorData[0]!=-1 & currentDirection==DIRECTION.WEST){
            distance = sensorData[0];
        }

        if(distance==-1 || distance>RobotConstants.CAMERA_RANGE) return;

        int cellRow = currentRow-1-distance;
        if(cellRow<0) return;

        int index = DIRECTION.NORTH.getIndex();
        if(exploredMap.getCell(cellRow, currentColumn).getIsProcessed(index)) return;

        exploredMap.getCell(cellRow, currentColumn).setIsProcessed(index, true);

        turnCameraDirection(DIRECTION.SOUTH);

        if(takePhoto(cellRow, currentColumn, DIRECTION.NORTH.toString())) foundImage++;;
        takenImage++;

        turnBotDirection(currentDirection);
    }

    /**
     * Returns true if the West cell has an obstacle
     */
    private void westProcess(){
        DIRECTION currentDirection = bot.getRobotCurDir();

        // We need to consider the possibility the robot is along the wall (so we can ignore the wall as an obstacle)
        int currentRow = bot.getRobotPosRow();
        int currentColumn = bot.getRobotPosCol();

        int distance=-1;
        // We consider different scenarios where the West side may be blocked
        if (sensorData[2]!=-1 & currentDirection==DIRECTION.WEST){
            distance = sensorData[2];
        }
        if (sensorData[0]!=-1 & currentDirection==DIRECTION.NORTH){
            distance=sensorData[0];
        }
        if (sensorData[4]!=-1 & currentDirection==DIRECTION.SOUTH){
            distance=sensorData[4];
        }

        if(distance==-1 || distance>RobotConstants.CAMERA_RANGE) return;

        int cellColumn = currentColumn-1-distance;
        if(cellColumn<0) return;

        int index = DIRECTION.EAST.getIndex();
        if(exploredMap.getCell(currentRow, cellColumn).getIsProcessed(index)) return;

        exploredMap.getCell(currentRow, cellColumn).setIsProcessed(index, true);

        turnCameraDirection(DIRECTION.WEST);

        if(takePhoto(currentRow, cellColumn, DIRECTION.EAST.toString())) foundImage++;;
        takenImage++;

        turnBotDirection(currentDirection);
    }


    public boolean takePhoto(int targetRow, int targetCol, String side){
        String coordinate = String.valueOf(targetRow) + "," + String.valueOf(targetCol);

        // TODO: Signal to Camera
        if(bot.getRealBot()){
            CommMgr.getCommMgr().sendMsg(coordinate, CommMgr.IR);
            CommMgr.getCommMgr().recvMsg();
        }
        else System.out.println("Take Photo: " + coordinate + " ("+ side +")");

        return false;
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
     * Returns the number of sides possible to have image attached.
     */
    private int calculateSidesPossible() {
        int result = 0;
        for (int r = 0; r < MapConstants.MAP_ROWS; r++) {
            for (int c = 0; c < MapConstants.MAP_COLS; c++) {
                if(exploredMap.getCell(r,c).getIsExplored() && exploredMap.getCell(r,c).getIsObstacle()){
                    if (r-1>=0 && exploredMap.getCell(r-1, c).getIsExplored() && !exploredMap.getCell(r-1, c).getIsObstacle()) {
                        result++;
                    }
                    if (r+1<MapConstants.MAP_ROWS && exploredMap.getCell(r+1, c).getIsExplored() && !exploredMap.getCell(r+1, c).getIsObstacle()) {
                        result++;
                    }
                    if (c-1>=0 && exploredMap.getCell(r, c-1).getIsExplored() && !exploredMap.getCell(r, c-1).getIsObstacle()) {
                        result++;
                    }
                    if (c+1<MapConstants.MAP_COLS &&exploredMap.getCell(r, c+1).getIsExplored() && !exploredMap.getCell(r, c+1).getIsObstacle()) {
                        result++;
                    }
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
     * Checks if the robot can calibrate at its current position given a direction.
     */
    private boolean canCalibrateOnTheSpot(DIRECTION botDir) {
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
        DIRECTION origDir = bot.getRobotCurDir();

        turnBotDirection(targetDir);
        moveBot(MOVEMENT.CALIBRATE);
        turnBotDirection(origDir);
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

    /**
     * Turns the front camera to the required direction.
     */
    private void turnCameraDirection(DIRECTION targetDir) {
        turnBotDirection(targetDir);
    }
}
