from flask import Flask, jsonify, send_from_directory
import os
from flask import render_template, url_for, make_response, request, session, redirect, flash, jsonify
from util import *

# Init app
app = Flask(__name__)
app.secret_key = 'DHCNHN'

# Main page
@app.route('/')
def index():
    return render_template('index.html')

# Login page
@app.route('/login')
def login_page():
    return render_template('login.html')

# API list
@app.route('/login', methods=['POST', 'GET'])
def Login():
    if request.method == 'POST':
        username = request.form['username'].strip()
        password = request.form['password'].strip()
        login_status = IsValidLogin(username, password)

        if login_status == True:
            return redirect(url_for('index'))
        else:
            flash('Invalid username or password', 'error')

    return render_template('login.html')

# Định nghĩa đường dẫn tới thư mục chứa ảnh
IMAGE_FOLDER = os.path.join(os.getcwd(), 'static/resources')

@app.route('/get-images', methods=['GET'])
def get_images():
    try:
        # Lấy tất cả các tệp trong thư mục resources
        files = os.listdir(IMAGE_FOLDER)
        
        # Lọc chỉ những tệp ảnh (jpg, jpeg, png, gif)
        images = [file for file in files if file.lower().endswith(('jpg', 'jpeg', 'png', 'gif'))]
        
        # Trả về danh sách ảnh dưới dạng JSON
        return jsonify(images)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# Cung cấp các tệp tĩnh như hình ảnh từ thư mục 'static/resources'
@app.route('/static/resources/<filename>')
def send_image(filename):
    return send_from_directory(IMAGE_FOLDER, filename)

app.run(debug=True, host = "0.0.0.0")

