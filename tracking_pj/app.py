import os
from flask import Flask, render_template
from ament_index_python.packages import get_package_share_directory
import rclpy
import threading

def create_app():
    share_dir = get_package_share_directory('tracking_pj')
    app = Flask(__name__,
                template_folder=os.path.join(share_dir, 'templates'),
                static_folder=os.path.join(share_dir, 'static'))

    @app.route('/')
    def index():
        return render_template('index.html')
    return app

def main():
    rclpy.init()
    app = create_app()
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000)).start()
    # 노드 생략 가능 (최소 실행)

if __name__ == '__main__':
    main()
