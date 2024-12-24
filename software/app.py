from flask import Flask
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

app.run(debug=True, host = "0.0.0.0")

