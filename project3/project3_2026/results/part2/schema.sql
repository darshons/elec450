CREATE TABLE experiments
        (id INTEGER PRIMARY KEY AUTOINCREMENT, name VARCHAR(512),
        totaltime REAL, timelimit REAL, memorylimit REAL, runcount INTEGER,
        version VARCHAR(128), hostname VARCHAR(1024), cpuinfo TEXT,
        date DATETIME, seed VARCHAR(24), setup TEXT);
CREATE TABLE sqlite_sequence(name,seq);
CREATE TABLE plannerConfigs
        (id INTEGER PRIMARY KEY AUTOINCREMENT,
        name VARCHAR(512) NOT NULL, settings TEXT);
CREATE TABLE enums
        (name VARCHAR(512), value INTEGER, description TEXT,
        PRIMARY KEY (name, value));
CREATE TABLE runs
        (id INTEGER PRIMARY KEY AUTOINCREMENT, experimentid INTEGER, plannerid INTEGER, approximate_solution BOOLEAN, correct_solution BOOLEAN, correct_solution_strict BOOLEAN, graph_motions INTEGER, graph_states INTEGER, memory REAL, minimum_clearance REAL, solution_clearance REAL, solution_difference REAL, solution_length REAL, solution_segments INTEGER, solution_smoothness REAL, solved BOOLEAN, status ENUM, time REAL, valid_segment_fraction REAL, approx_goal_distance REAL, best_cost REAL, edge_count INTEGER, iterations INTEGER, milestone_count INTEGER,
        FOREIGN KEY (experimentid) REFERENCES experiments(id) ON DELETE CASCADE,
        FOREIGN KEY (plannerid) REFERENCES plannerConfigs(id) ON DELETE CASCADE);
CREATE TABLE progress
        (runid INTEGER, time REAL, best_cost REAL, edge_count INTEGER, iterations INTEGER, milestone_count INTEGER, PRIMARY KEY (runid, time),
        FOREIGN KEY (runid) REFERENCES runs(id) ON DELETE CASCADE);
CREATE VIEW bestPlannerConfigsPerExperiment AS SELECT plannerid, experimentid, MIN(avg_solved) AS avg_solved, avg_total_time
        FROM (SELECT plannerid, plannerName, experimentid, AVG(solved) AS avg_solved,
        AVG(total_time) AS avg_total_time
        FROM (SELECT plannerid, plannerConfigs.name AS plannerName, experimentid,
            solved, time AS total_time
            FROM plannerConfigs INNER JOIN experiments INNER JOIN runs
            ON plannerConfigs.id=runs.plannerid AND experiments.id=runs.experimentid) GROUP BY plannerid, experimentid) GROUP BY plannerName, experimentid ORDER BY avg_solved DESC,
        avg_total_time ASC;
CREATE VIEW bestPlannerConfigs AS SELECT plannerid, MIN(avg_solved) AS avg_solved, avg_total_time
        FROM (SELECT plannerid, plannerName, AVG(solved) AS avg_solved,
        AVG(total_time) AS avg_total_time
        FROM (SELECT plannerid, plannerConfigs.name AS plannerName, experimentid,
            solved, time AS total_time
            FROM plannerConfigs INNER JOIN experiments INNER JOIN runs
            ON plannerConfigs.id=runs.plannerid AND experiments.id=runs.experimentid) GROUP BY plannerid) GROUP BY plannerName ORDER BY avg_solved DESC, avg_total_time ASC;