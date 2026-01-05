package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.stream.Collectors;

/**
 * Log Command Scheduler actions for command initialize, execute, interrupt, finish
 * <p>This class could be used this way:
 *
<pre><code>
    // class variable
    private CommandSchedulerLog schedulerLog;

    // constructor statements
    // Select 1 or more different Command logging methods as desired
    configCommandLogs(EnumSet.of(LogsSelector.useConsole, LogsSelector.useDataLog));
    // or none
    // configCommandLogs(EnumSet.noneOf(LogsSelector.class));

    /**
     * Configure Command logging to Console/Terminal, DataLog, or ShuffleBoard
     {@literal*}/
    @SuppressWarnings("resource")
    public void configCommandLogs(EnumSet<LogsSelector> logsSelector)
    {
        if ( ! logsSelector.isEmpty())
         {
            schedulerLog = new CommandSchedulerLog(logsSelector);
            schedulerLog.logCommandInitialize();
            schedulerLog.logCommandInterrupt();
            schedulerLog.logCommandFinish();
            schedulerLog.logCommandExecute();  // Can (optionally) generate a lot of output        
        }
        else {
            new Alert("No logging", AlertType.kWarning).set(true);
        }
    }
</code></pre>
 */
public class CommandSchedulerLog 
{
    static
    {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    public enum LogsSelector {useConsole, useDataLog, useShuffleBoardLog};

    private final HashMap<String, Integer> m_currentCommands = new HashMap<String, Integer>();
    private final NetworkTable m_nt;    
    private final StringEntry m_initializeCommandLogEntry;
    private final StringEntry m_interruptCommandLogEntry;
    private final StringEntry m_finishCommandLogEntry;
    private final StringEntry m_executeCommandLogEntry;
    private EnumSet<LogsSelector> logsSelector;

    /**
     * Command Event Loggers
     * 
     * <p>Set the command scheduler to log events for command initialize, interrupt, finish, and execute.
     * 
     * <p>Log to the Console/Terminal, ShuffleBoard or the WPILib DataLog tool.
     * 
     * <p>If ShuffleBoard is recording (start it manually), these events are added to the recording.
     * Convert recording to csv and they show nicely in Excel.
     * 
     * <p>If using DataLog tool, the recording is via NT so tell NT to send EVERYTHING to the DataLog.
     * Run DataLog tool to retrieve log from roboRIO and convert the log to a csv table that may be
     * viewed nicely in Excel.
     * 
     * <p>Note the comment in execute logging that only the first execute is logged unless changed.
     * 
     * <p>Note that use of the DataLog creates SmartDashboard/ShuffleBoard entries but are not the same
     * as use of the ShuffleBoardLog. The ShuffleBoardLog has event markers independent of the DataLog.
     * 
     * @param useConsole
     * @param useDataLog
     * @param useShuffleBoardLog
     */ 
    public CommandSchedulerLog(EnumSet<LogsSelector> logsSelector)
    {
        this.logsSelector = logsSelector;

        // DataLog via NT so establish NT and the connection to DataLog
        if (logsSelector.contains(LogsSelector.useDataLog)) {
            DataLogManager.logNetworkTables(true); // CAUTION - this puts all NT to the DataLog
        }

        final String networkTableName = "CommandLog";
        m_nt = NetworkTableInstance.getDefault().getTable(networkTableName);
        m_initializeCommandLogEntry = m_nt.getStringTopic("Commands/initialize").getEntry("");
        m_interruptCommandLogEntry = m_nt.getStringTopic("Commands/interrupt").getEntry("");
        m_finishCommandLogEntry = m_nt.getStringTopic("Commands/finish").getEntry("");
        m_executeCommandLogEntry = m_nt.getStringTopic("Commands/execute").getEntry("");        
    }

    /**
     * Log commands that run the initialize method.
     */
    public void logCommandInitialize()
    {
        CommandScheduler.getInstance().onCommandInitialize(
            (command) -> 
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();
                String requirements = command.getRequirements().stream()
                    .map(subsystem -> subsystem.getClass().getSimpleName())
                    .collect(Collectors.joining(", ", "{", "}"));

                if (logsSelector.contains(LogsSelector.useConsole)) {
                    System.out.println("Command initialized : " + key + " " + requirements);                    
                }
                if (logsSelector.contains(LogsSelector.useDataLog)) {
                    m_initializeCommandLogEntry.set(key + " " + requirements);                    
                }
                if (logsSelector.contains(LogsSelector.useShuffleBoardLog)) {
                    Shuffleboard.addEventMarker("Command initialized",
                        key + " " + requirements, EventImportance.kNormal);                    
                }

                m_currentCommands.put(key, 0);
            }
        );
    }

    /**
     * Log commands that have been interrupted.
     */
    public void logCommandInterrupt()
    {
        CommandScheduler.getInstance().onCommandInterrupt(
            (command, interruptedBy) ->
            {
                String interrupter;
                if (interruptedBy.isPresent()) {
                    interrupter =  
                        "interrupted by command " + command.getClass().getSimpleName() + "/" + interruptedBy.get().getName();
                }
                else {
                    interrupter = 
                        "interrupted"; // interrupted not by a command - mode change, cancelled, timeOut, until, etc.
                }

                String key = command.getClass().getSimpleName() + "/" + command.getName();
                String runs = " after " + m_currentCommands.getOrDefault(key, 0) + " runs " + interrupter;

                if (logsSelector.contains(LogsSelector.useConsole)) {
                    System.out.println(key + runs);                    
                }
                if (logsSelector.contains(LogsSelector.useDataLog)) {
                    m_interruptCommandLogEntry.set(key + runs);                    
                } 
                if (logsSelector.contains(LogsSelector.useShuffleBoardLog)) {
                    Shuffleboard.addEventMarker("Command interrupted", key + runs, EventImportance.kNormal);
                }

                m_currentCommands.put(key, 0);
            }
        );
    }

    /**
     * Log commands that run the finish method.
     */
    public void logCommandFinish()
    {
        CommandScheduler.getInstance().onCommandFinish(
            (command) ->
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();
                String runs = " after " + m_currentCommands.getOrDefault(key, 0) + " runs";

                if (logsSelector.contains(LogsSelector.useConsole)) {
                    System.out.println("Command finished : " + key + runs);                    
                }
                if (logsSelector.contains(LogsSelector.useDataLog)) {
                    m_finishCommandLogEntry.set(key + runs);                    
                } 
                if (logsSelector.contains(LogsSelector.useShuffleBoardLog)) {
                    Shuffleboard.addEventMarker("Command finished", key, EventImportance.kNormal);                    
                }

                m_currentCommands.put(key, 0);
            }
        );
    }

    /**
     * Log commands that run the execute() method.
     * 
     * <p>This can generate a lot of events so logging is suppressed except for the first
     * occurrence of execute(). Total count of execute() is logged at command end.
     * 
     * <p>Recompile without the if/else to get all execute() logged.
     */
    public void logCommandExecute()
    {
        CommandScheduler.getInstance().onCommandExecute(
            (command) ->
            {
                String key = command.getClass().getSimpleName() + "/" + command.getName();

                if (m_currentCommands.getOrDefault(key, 0) == 0) // suppress all but first execute
                {
                    if (logsSelector.contains(LogsSelector.useConsole)) {
                        System.out.println("Command executed : " + key);                        
                    }
                    if (logsSelector.contains(LogsSelector.useDataLog)) {
                        m_executeCommandLogEntry.set(key);             
                    }
                    if (logsSelector.contains(LogsSelector.useShuffleBoardLog)) {
                        Shuffleboard.addEventMarker("Command executed", key, EventImportance.kNormal);                        
                    }

                    m_currentCommands.put(key, 1); // first time through count is 1
                }
                else
                {
                    // Increment total count to log when the command ends.
                    m_currentCommands.put(key, m_currentCommands.get(key) + 1);
                }
            }
        );
    }
}
