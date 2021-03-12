package runner;

import algorithms.ExplorationAlgo;
import algorithms.FastestPathAlgo;
import map.Map;
import robot.Robot;
import robot.RobotConstants;
import utils.CommMgr;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

import static utils.MapDescriptor.*;

/**
 * Simulator for robot navigation in virtual arena.
 */

public class MazeRunner {
    private static JFrame _appFrame = null;         // application JFrame

    private static JPanel _mapCards = null;         // JPanel for map views
    private static JPanel _buttons = null;          // JPanel for buttons

    private static Robot bot;

    private static Map realMap = null;              // real map
    private static Map exploredMap = null;          // exploration map

    private static int timeLimit = 3600;            // time limit
    private static int coverageLimit = 300;         // coverage limit

    private static final CommMgr comm = CommMgr.getCommMgr();

    private static final String filename = "MD6";
    private static int waypointRow = 1;
    private static int waypointCol = 1;

    private static final boolean explorationMode = false;
    private static final boolean pledgeEnabled = false;

    /**
     * Initialises the different maps and displays the application.
     */
    public static void main(String[] args) {
        comm.openConnection();
        bot = new Robot(RobotConstants.START_ROW, RobotConstants.START_COL, explorationMode);

        realMap = new Map(bot);
        realMap.setAllUnexplored();

        exploredMap = new Map(bot);
        exploredMap.setAllUnexplored();

        displayEverything();
    }

    /**
     * Initialises the different parts of the application.
     */
    private static void displayEverything() {
        // Initialise main frame for display
        _appFrame = new JFrame();
        _appFrame.setTitle("Runner");
        _appFrame.setSize(new Dimension(700, 700));
        _appFrame.setResizable(false);

        // Center the main frame in the middle of the screen
        Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();
        _appFrame.setLocation(dim.width / 2 - _appFrame.getSize().width / 2, dim.height / 2 - _appFrame.getSize().height / 2);

        // Create the CardLayout for storing the different maps
        _mapCards = new JPanel(new CardLayout());

        // Create the JPanel for the buttons
        _buttons = new JPanel();

        // Add _mapCards & _buttons to the main frame's content pane
        Container contentPane = _appFrame.getContentPane();
        contentPane.add(_mapCards, BorderLayout.CENTER);
        contentPane.add(_buttons, BorderLayout.PAGE_END);

        // Initialize the main map view
        initMainLayout();

        // Initialize the buttons
        initButtonsLayout();

        // Display the application
        _appFrame.setVisible(true);
        _appFrame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
    }

    /**
     * Initialises the main map view by adding the different maps as cards in the CardLayout. Displays realMap
     * by default.
     */
    private static void initMainLayout() {
        _mapCards.add(realMap, "REAL_MAP");
        _mapCards.add(exploredMap, "EXPLORATION");

        CardLayout cl = ((CardLayout) _mapCards.getLayout());
        cl.show(_mapCards, "EXPLORATION");
    }

    /**
     * Initialises the JPanel for the buttons.
     */
    private static void initButtonsLayout() {
        _buttons.setLayout(new GridLayout());
        addButtons();
    }

    /**
     * Helper method to set particular properties for all the JButtons.
     */
    private static void formatButton(JButton btn) {
        btn.setFont(new Font("Arial", Font.BOLD, 13));
        btn.setFocusPainted(false);
    }

    /**
     * Initialises and adds the five main buttons. Also creates the relevant classes (for multithreading) and JDialogs
     * (for user input) for the different functions of the buttons.
     */
    private static void addButtons() {
        // Fastest Path Button
        JButton btn_FastestPath = new JButton("Fastest Path");
        formatButton(btn_FastestPath);
        btn_FastestPath.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                loadMap(realMap, filename);
                realMap.repaint();

                CardLayout cl = ((CardLayout) _mapCards.getLayout());
                cl.show(_mapCards, "REAL_MAP");

                new FastestPath().execute();
            }
        });
        _buttons.add(btn_FastestPath);

        // Exploration Button
        JButton btn_Exploration = new JButton("Exploration");
        formatButton(btn_Exploration);
        btn_Exploration.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                new Exploration().execute();
            }
        });
        _buttons.add(btn_Exploration);

        // Image Exploration Button
        JButton btn_Image_Exploration = new JButton("Image Processing");
        formatButton(btn_Image_Exploration);
        btn_Image_Exploration.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                new ImageProcessing().execute();
            }
        });
        _buttons.add(btn_Image_Exploration);
    }

    // FastestPath Class for Multithreading
    private static class FastestPath extends SwingWorker<Integer, String> {
        protected Integer doInBackground() throws Exception {
            bot.setRobotPos(RobotConstants.START_ROW, RobotConstants.START_COL);
            bot.setRobotDir(RobotConstants.DIRECTION.NORTH);

            String msg = comm.recvMsg();
            if(!msg.isEmpty()){
                waypointRow =Integer.parseInt(msg.substring(0, msg.indexOf(',')));
                waypointCol =Integer.parseInt(msg.substring(msg.indexOf(',')+1));
            }

            realMap.setWaypoint(waypointRow, waypointCol);
            realMap.repaint();

            FastestPathAlgo fastestPathToWayPoint;
            fastestPathToWayPoint = new FastestPathAlgo(realMap, bot);
            String output1 = fastestPathToWayPoint.runFastestPath(waypointRow, waypointCol);

            bot.setRobotPos(waypointRow, waypointCol);

            FastestPathAlgo fastestPathToGoal;
            fastestPathToGoal = new FastestPathAlgo(realMap, bot);
            String output2 = fastestPathToGoal.runFastestPath(RobotConstants.GOAL_ROW, RobotConstants.GOAL_COL);

            String output;
            char output1End = output1.charAt(output1.length()-1);
            char output2Begin = output2.charAt(0);

            if(output1End<='8' && output1End>='1' && output2Begin<='8' && output2Begin>='1'
                    && output1End+output2Begin-'0'<='9' && output1End+output2Begin-'0'>='1'){

                output = output1.substring(0, output1.length()-1)
                        + Character.toString(output1End+output2Begin-'0')
                        + output2.substring(1);
            }
            else{
                output = output1+output2;
            }

            comm.sendMsg(output, CommMgr.AR);
            comm.sendMsg("FP_START", CommMgr.AN);

            return 111;
        }
    }

    // Exploration Class for Multithreading
    private static class Exploration extends SwingWorker<Integer, String> {
        protected Integer doInBackground() throws Exception {
            bot.setRobotPos(RobotConstants.START_ROW, RobotConstants.START_COL);

            ExplorationAlgo exploration;
            exploration = new ExplorationAlgo(exploredMap, realMap, bot, coverageLimit, timeLimit);

            exploration.setPledgeEnabled(pledgeEnabled);

            exploration.runExploration();

            return 222;
        }
    }

    // Image Exploration Class for Multithreading
    private static class ImageProcessing extends SwingWorker<Integer, String> {
        protected Integer doInBackground() throws Exception {
            bot.setRobotPos(RobotConstants.START_ROW, RobotConstants.START_COL);

            ExplorationAlgo image;
            image = new ExplorationAlgo(exploredMap, realMap, bot, coverageLimit, timeLimit);

            image.setPledgeEnabled(pledgeEnabled);
            image.setImageProcessing(true);

            image.runExploration();
            generateMapDescriptor(exploredMap);

            return 333;
        }
    }
}
