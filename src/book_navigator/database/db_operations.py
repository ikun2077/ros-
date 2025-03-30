#!/usr/bin/env python3
import sqlite3
from datetime import datetime

class BookDatabase:
    def __init__(self, db_path='database/book_db.sqlite3'):
        self.conn = sqlite3.connect(db_path)
        self.create_tables()
        
    def create_tables(self):
        cursor = self.conn.cursor()
        # 书籍表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS books (
                id TEXT PRIMARY KEY,
                end_x REAL NOT NULL,
                end_y REAL NOT NULL,
                end_z REAL DEFAULT 0.0,
                end_yaw REAL DEFAULT 0.0,
                last_updated DATETIME DEFAULT CURRENT_TIMESTAMP
            )''')
        
        # 配置表
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS system_config (
                config_key TEXT PRIMARY KEY,
                config_value TEXT,
                description TEXT
            )''')
        self.conn.commit()
    
    def add_book(self, book_id, end_x, end_y, end_z=0.0, end_yaw=0.0):
        cursor = self.conn.cursor()
        try:
            cursor.execute('''
                INSERT OR REPLACE INTO books 
                (id, end_x, end_y, end_z, end_yaw)
                VALUES (?, ?, ?, ?, ?)''',
                (book_id, end_x, end_y, end_z, end_yaw))
            self.conn.commit()
            return True
        except sqlite3.Error as e:
            print(f"添加书籍失败: {str(e)}")
            return False
    
    def get_config(self, key):
        cursor = self.conn.cursor()
        cursor.execute('SELECT config_value FROM system_config WHERE config_key=?', (key,))
        result = cursor.fetchone()
        return float(result[0]) if result else None
    
    def backup_database(self, backup_dir='database/backups'):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_file = f"{backup_dir}/book_db_{timestamp}.bak"
        
        try:
            with open(backup_file, 'w') as f:
                for line in self.conn.iterdump():
                    f.write(f'{line}\n')
            print(f"数据库备份成功: {backup_file}")
            return True
        except Exception as e:
            print(f"备份失败: {str(e)}")
            return False

if __name__ == '__main__':
    db = BookDatabase()
    
    # 示例：添加测试数据
    test_books = [
        ('9787111636663', 3.5, 2.1, 0.0, 1.57),  # 机器人学导论
        ('9787302455106', 1.8, 4.2, 0.0, 0.0),    # ROS机器人开发
        ('9787121373994', 5.0, 3.0, 0.0, -0.78)   # 自动控制原理
    ]
    
    for book in test_books:
        if db.add_book(*book):
            print(f"成功添加书籍: {book[0]}")