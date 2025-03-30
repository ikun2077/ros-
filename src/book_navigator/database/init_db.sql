-- 书籍位置信息表
CREATE TABLE IF NOT EXISTS books (
    id TEXT PRIMARY KEY,        -- ISBN或唯一标识码
    end_x REAL NOT NULL,       -- X坐标（米）
    end_y REAL NOT NULL,       -- Y坐标（米）
    end_z REAL DEFAULT 0.0,    -- Z坐标（米）
    end_yaw REAL DEFAULT 0.0,  -- 朝向（弧度）
    last_updated DATETIME DEFAULT CURRENT_TIMESTAMP
);

-- 系统配置表
CREATE TABLE IF NOT EXISTS system_config (
    config_key TEXT PRIMARY KEY,
    config_value TEXT,
    description TEXT
);

-- 插入默认配置
INSERT OR IGNORE INTO system_config (config_key, config_value, description)
VALUES 
    ('start_x', '0.0', '默认起点X坐标'),
    ('start_y', '0.0', '默认起点Y坐标'),
    ('start_yaw', '0.0', '默认起点朝向');