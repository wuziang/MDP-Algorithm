package runner;

import algorithms.ExplorationAlgo;
import algorithms.FastestPathAlgo;
import algorithms.ImageExplorationAlgo;
import map.Map;
import map.MapConstants;
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

    private static int waypointX = 12;
    private static int waypointY = 9;

    private static int timeLimit = 3600;            // time limit
    private static int coverageLimit = 300;         // coverage limit

    private static final CommMgr comm = CommMgr.getCommMgr();
    private static final boolean realRun = false;

    private static final String filename = "MD1";

    /**
     * Initialises the different maps and displays the application.
     */
    public static void main(String[] args) {
        if (realRun) comm.openConnection();

        bot = new Robot(RobotConstants.START_ROW, RobotConstants.START_COL, true);

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
        _appFrame.setTitle("MDP Group 4");
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
        cl.show(_mapCards, "REAL_MAP");
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
        // FastestPath Class for Multithreading
        class FastestPath extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                bot.setRobotPos(RobotConstants.START_ROW, RobotConstants.START_COL);

                if (realRun) {
                    String msg = comm.recvMsg();
                    if(!msg.isEmpty()){
                        waypointX=Integer.parseInt(msg.substring(0, msg.indexOf(',')));
                        waypointY=Integer.parseInt(msg.substring(msg.indexOf(',')+1));
                    }
                }

                FastestPathAlgo fastestPathToWayPoint;
                fastestPathToWayPoint = new FastestPathAlgo(realMap, bot);
                String output1 = fastestPathToWayPoint.runFastestPath(waypointX,waypointY);

                bot.setRobotPos(waypointX,waypointY);

                FastestPathAlgo fastestPathToGoal;
                fastestPathToGoal = new FastestPathAlgo(realMap, bot);
                String output2 = fastestPathToGoal.runFastestPath(RobotConstants.GOAL_ROW, RobotConstants.GOAL_COL);

                comm.sendMsg(output1+output2, CommMgr.AR);
                comm.sendMsg("FP_START", CommMgr.AN);

                return 222;
            }
        }

        // Fastest Path Button
        JButton btn_FastestPath = new JButton("Fastest Path");
        formatButton(btn_FastestPath);
        btn_FastestPath.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
//                loadMapFromDisk(realMap, filename);
                loadMapDescriptorFromDisk(realMap, filename);
                realMap.repaint();

                new FastestPath().execute();
            }
        });
        _buttons.add(btn_FastestPath);

        // Exploration Class for Multithreading
        class Exploration extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                int row, col;

                row = RobotConstants.START_ROW;
                col = RobotConstants.START_COL;

                bot.setRobotPos(row, col);
                exploredMap.repaint();

                ExplorationAlgo exploration;
                exploration = new ExplorationAlgo(exploredMap, realMap, bot, coverageLimit, timeLimit);

                if (realRun) {
                    CommMgr.getCommMgr().sendMsg(null, CommMgr.AR);
                }

                exploration.runExploration();
                generateMapDescriptor(exploredMap);

                if (realRun) {
                    new FastestPath().execute();
                }

                return 111;
            }
        }

        // Image Exploration Class for Multithreading
        class ImageExploration extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                int row, col;

                row = RobotConstants.START_ROW;
                col = RobotConstants.START_COL;

                bot.setRobotPos(row, col);
                exploredMap.repaint();

                ImageExplorationAlgo image_exploration;
                image_exploration = new ImageExplorationAlgo(exploredMap, realMap, bot, coverageLimit, timeLimit);

                if (realRun) {
                    CommMgr.getCommMgr().sendMsg(null, CommMgr.AR);
                }

                image_exploration.runExploration();
                generateMapDescriptor(exploredMap);

                if (realRun) {
                    new FastestPath().execute();
                }

                return 333;
            }
        }

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
        JButton btn_Image_Exploration = new JButton("Image Exploration");
        formatButton(btn_Image_Exploration);
        btn_Image_Exploration.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                new ImageExploration().execute();
            }
        });
        _buttons.add(btn_Image_Exploration);
    }
}
