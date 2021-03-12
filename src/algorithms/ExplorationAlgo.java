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

    private int areaExplored;

    private final int coverageLimit;
    private final int timeLimit;

    private long startTime;
    private long endTime;

    private int lastCalibrate;
    private boolean calibrationMode;

    private int[] sensorData;

    private boolean imageProcessing;
    private int foundImage=0;
    private int takenImage=0;

    private boolean pledgeEnabled;
    private boolean pledgeMode;

    public ExplorationAlgo(Map exploredMap, Map realMap, Robot bot, int coverageLimit, int timeLimit) {
        this.exploredMap = exploredMap;
        this.realMap = realMap;
        this.bot = bot;
        this.coverageLimit = coverageLimit;
        this.timeLimit = timeLimit;
    }

    public void setImageProcessing(boolean imageProcessing){
        this.imageProcessing = imageProcessing;
    }

    public void setPledgeEnabled(boolean pledgeEnabled){
        this.pledgeEnabled = pledgeEnabled;
    }

    /**
     * Main method that is called to start the exploration.
     */
    public void runExploration() {
        System.out.println("\nExploring...");

        if (bot.getRealBot()) {
            // TODO:: Receive start from Android
//            CommMgr.getCommMgr().recvMsg();

            CommMgr.getCommMgr().sendMsg("S", CommMgr.AR);
        }

        startTime = System.currentTimeMillis();
        endTime = startTime + (timeLimit * 1000);

        senseAndUpdate();
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
                if(areaExplored>=100) {
                    break;
                }
            }

            if(pledgeEnabled) {
                runPledgeAlgo();
            }

        } while (areaExplored <= coverageLimit && System.currentTimeMillis() <= endTime);

        exploredMap.repaint();

        if(!imageProcessing) {
            areaExplored = calculateAreaExplored();
            System.out.printf("\nExploration Coverage %.2f%%\n", (areaExplored / 300.0) * 100.0);

            exploredMap.guessUnexploredCells();
            String[] mapStrings = MapDescriptor.generateMapDescriptor(exploredMap);
            System.out.println("P1: " + mapStrings[0]);
            System.out.println("P2: " + mapStrings[1]);
        }
        else {
            double sides = calculateSidesPossible();
            System.out.printf("\nImage Processing Coverage %.2f%%", (takenImage / sides) * 100.0);

            if (bot.getRealBot()) {
                System.out.printf("\nImages Found: %d\n", foundImage);
            }
        }
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
     * Pledge algorithm
     * General flow:
     * - If an obstacle is detected to the robot's left/right, begin pledge algorithm
     * - This will be inside a while loop (or maybe not) and will continue until the very starting waypoint is reached again
     * - The starting waypoint should not be visited and this should be the main condition to start the pledge algorithm
     **/
    private void runPledgeAlgo() {
        pledgeMode=true;
        if (bot.getRobotPosCol() != 1 & bot.getRobotPosRow() != 1) {
            int currentColumn = 0;
            int currentRow = 0;
            DIRECTION startingDirection = bot.getRobotCurDir();

            if (sensorData[0] == 1){
                int startColumn = bot.getRobotPosCol();
                int startRow = bot.getRobotPosRow();

                if(exploredMap.getCell(startRow, startColumn).getIsPledged()) return;

                boolean flag = false;

                moveBot(MOVEMENT.BACKWARD);

                // For increase if the robot is still stuck
                switch (startingDirection) {
                    case NORTH:
                        turnBotDirection(DIRECTION.WEST);
                        break;
                    case EAST:
                        turnBotDirection(DIRECTION.NORTH);
                        break;
                    case SOUTH:
                        turnBotDirection(DIRECTION.EAST);
                        break;
                    case WEST:
                        turnBotDirection(DIRECTION.SOUTH);
                        break;
                }

                while (sensorData[1]!=-1 & sensorData[1]<=2 | sensorData[2]!=-1 & sensorData[2]<=2 | sensorData[3]!=-1 & sensorData[3]<=2){
                    turnBotDirection(startingDirection);
                    moveBot(MOVEMENT.BACKWARD);
                    switch (startingDirection) {
                        case NORTH:
                            turnBotDirection(DIRECTION.WEST);
                            break;
                        case EAST:
                            turnBotDirection(DIRECTION.NORTH);
                            break;
                        case SOUTH:
                            turnBotDirection(DIRECTION.EAST);
                            break;
                        case WEST:
                            turnBotDirection(DIRECTION.SOUTH);
                            break;
                    }
                }

                // Make the bot move forward so it will have an obstacle on it's right side
                moveBot(MOVEMENT.FORWARD);
                moveBot(MOVEMENT.FORWARD);

                // Then we start making the robot traverse across the perimeter of the obstacle
                while (flag == false) {
                    currentColumn = bot.getRobotPosCol();
                    currentRow = bot.getRobotPosRow();
                    DIRECTION currentDirection = bot.getRobotCurDir();

                    switch (currentDirection) {
                        case NORTH:
                            if (lookRight()) {
                                turnBotDirection(DIRECTION.EAST);
                                moveBot(MOVEMENT.FORWARD);
                            } else if (sensorData[4] > -1 & lookForward()) {
                                moveBot(MOVEMENT.FORWARD);
                            } else if (sensorData[4] > -1 & lookLeft() & !lookForward()) {
                                turnBotDirection(DIRECTION.WEST);
                            } else {
                                if (currentColumn!=1 & currentColumn!=13){
                                    moveBot(MOVEMENT.FORWARD);
                                }
                                else{
                                    if (!lookForward()){
                                        flag = true;
                                        break;
                                    }
                                    moveBot(MOVEMENT.FORWARD);
                                }
                            }
                            break;
                        case EAST:
                            if (lookRight()) {
                                turnBotDirection(DIRECTION.SOUTH);
                                moveBot(MOVEMENT.FORWARD);
                            } else if (sensorData[4] > -1 & lookForward()) {
                                moveBot(MOVEMENT.FORWARD);
                            } else if (sensorData[4] > -1 & lookLeft() & !lookForward() & currentColumn!=13) {
                                turnBotDirection(DIRECTION.NORTH);
                            } else {
                                if (currentColumn!=1 & currentColumn!=13){
                                    moveBot(MOVEMENT.FORWARD);
                                }
                                else{
                                    if (!lookForward()){
                                        flag = true;
                                        break;
                                    }
                                    moveBot(MOVEMENT.FORWARD);
                                }
                            }
                            break;
                        case SOUTH:
                            if (lookRight()) {
                                turnBotDirection(DIRECTION.WEST);
                                moveBot(MOVEMENT.FORWARD);
                            } else if (sensorData[4] > -1 & lookForward()) {
                                moveBot(MOVEMENT.FORWARD);
                            } else if (sensorData[4] > -1 & lookLeft() & !lookForward()) {
                                turnBotDirection(DIRECTION.EAST);
                            } else {
                                if (currentColumn!=1 & currentColumn!=13){
                                    moveBot(MOVEMENT.FORWARD);
                                }
                                else{
                                    if (!lookForward()){
                                        flag = true;
                                        break;
                                    }
                                    moveBot(MOVEMENT.FORWARD);
                                }
                            }
                            break;
                        case WEST:
                            if (lookRight()) {
                                turnBotDirection(DIRECTION.NORTH);
                                moveBot(MOVEMENT.FORWARD);
                            } else if (sensorData[4] > -1 & lookForward()) {
                                moveBot(MOVEMENT.FORWARD);
                            } else if (sensorData[4] > -1 & lookLeft() & !lookForward() & currentColumn!=1){
                                turnBotDirection(DIRECTION.SOUTH);
                            } else {
                                if (currentColumn!=1 & currentColumn!=13){
                                    moveBot(MOVEMENT.FORWARD);
                                }
                                else{
                                    if (!lookForward()){
                                        flag = true;
                                        break;
                                    }
                                    moveBot(MOVEMENT.FORWARD);
                                }
                            }
                            break;
                    }

                    if (currentColumn!=1 | currentColumn!=13){
                        if (currentColumn == startColumn & currentRow == startRow) {
                            // Make Robot face the front once again
                            switch (currentDirection) {
                                case NORTH:
                                    turnBotDirection(DIRECTION.SOUTH);
                                    break;
                                case EAST:
                                    turnBotDirection(DIRECTION.WEST);
                                    break;
                                case SOUTH:
                                    turnBotDirection(DIRECTION.NORTH);
                                    break;
                                case WEST:
                                    turnBotDirection(DIRECTION.EAST);
                                    break;
                            }

                            while (sensorData[0]!=-1){
                                moveBot(MOVEMENT.FORWARD);
                            }
                            moveBot(MOVEMENT.FORWARD);

                            flag = true;
                        }
                    }

                    if ((currentColumn==1 | currentColumn==13) & currentColumn == startColumn & !lookForward()){
                        // Make Robot face the front once again
                        switch (currentDirection) {
                            case NORTH:
                                turnBotDirection(DIRECTION.SOUTH);
                                break;
                            case EAST:
                                turnBotDirection(DIRECTION.WEST);
                                break;
                            case SOUTH:
                                turnBotDirection(DIRECTION.NORTH);
                                break;
                            case WEST:
                                turnBotDirection(DIRECTION.EAST);
                                break;
                            }
                        flag = true;
                    }
                }
            }
        }
        pledgeMode=false;
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
        exploredMap.repaint();
        bot.move(m);

        if (m != MOVEMENT.CALIBRATE) {
            senseAndUpdate();
        }

        if (bot.getRealBot() && !calibrationMode) {
            calibrationMode = true;
            bot.setInCalibration(true);

            if (canCalibrateOnTheSpot(bot.getRobotCurDir())) {
                lastCalibrate = 0;
                moveBot(MOVEMENT.CALIBRATE);
            } else {
                lastCalibrate++;
                if (lastCalibrate >= RobotConstants.CALIBRATE_THRESHOLD) {
                    DIRECTION targetDir = getCalibrationDirection();
                    if (targetDir != null) {
                        lastCalibrate = 0;
                        calibrateBot(targetDir);
                    }
                }
            }

            calibrationMode = false;
            bot.setInCalibration(false);
        }

        if(imageProcessing){
            northImage();
            eastImage();
            southImage();
            westImage();
        }

        if(pledgeMode){
            exploredMap.getCell(bot.getRobotPosRow(), bot.getRobotPosCol()).setIsPledged(true);
        }
    }

    /**
     * Sets the bot's sensors, processes the sensor data and repaints the map.
     */
    private void senseAndUpdate() {
        bot.setSensors();
        sensorData = bot.sense(exploredMap, realMap);

        if(!imageProcessing) {
            sendToAndroid();
        }
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

    private void northImage(){
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

        if(sendToCamera(cellRow, currentColumn, DIRECTION.SOUTH.toString())) foundImage++;;
        takenImage++;

        turnBotDirection(currentDirection);
    }

    private void eastImage(){
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

        if(sendToCamera(currentRow, cellColumn, DIRECTION.WEST.toString())) foundImage++;
        takenImage++;

        turnBotDirection(currentDirection);
    }

    private void southImage(){
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

        if(sendToCamera(cellRow, currentColumn, DIRECTION.NORTH.toString())) foundImage++;;
        takenImage++;

        turnBotDirection(currentDirection);
    }

    private void westImage(){
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

        if(sendToCamera(currentRow, cellColumn, DIRECTION.EAST.toString())) foundImage++;;
        takenImage++;

        turnBotDirection(currentDirection);
    }

    private void turnCameraDirection(DIRECTION targetDir) {
        turnBotDirection(targetDir);
    }

    private void sendToAndroid(){
        String[] mapStrings = MapDescriptor.generateMapDescriptor(exploredMap);
        String robotRow = String.valueOf(bot.getRobotPosRow());
        String robotCol = String.valueOf(bot.getRobotPosCol());
        String robotDir = Character.toString(DIRECTION.print(bot.getRobotCurDir()));

        if(bot.getRealBot()){
            CommMgr.getCommMgr().sendMsg(mapStrings[0] + "," + mapStrings[1] + "," + robotCol + "," + robotRow + "," + robotDir, CommMgr.AN);
        }
    }

    private boolean sendToCamera(int targetRow, int targetCol, String side){
        String coordinate = targetRow + "," + targetCol;

        if(bot.getRealBot()){
            CommMgr.getCommMgr().sendMsg(coordinate, CommMgr.IR);
            CommMgr.getCommMgr().recvMsg();
        }
        else System.out.println("Take Photo: " + coordinate + " ("+ side +")");

        return false;
    }
}
