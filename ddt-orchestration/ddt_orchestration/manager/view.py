import flask
from flask import Flask, render_template, request, session, redirect, url_for, Markup, jsonify
from flask_socketio import SocketIO, send, join_room, leave_room, emit, rooms
import json as Json
import logging
from threading import Lock
from pathlib import Path
from functools import partial

from ddt_orchestration.manager.action import make_ros_graph, pause_ros_graph, stop_command, pre_make_ros_graph
from ddt_orchestration.model.ddt_model import Application, Pod, Message, Process

app = Flask(__name__)

app.config['SECRET_KEY'] = 'secret'
app.config['DEBUG'] = True

socketio = SocketIO(app)
thread = None
thread_lock = Lock()
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

AppNames = list()
Apps = list()
# public room
RoomWeb = list()
RoomWebName = 'Room-Web'
WebID = None
ManagerPwd = Path(__file__).parent.parent
TmpFolder = ManagerPwd / 'tmp'
Processes = ['rosgraph', 'debug_bridge', 'rosgraph_bridge']

def show_ros_graph(app):
    app_path = Path(TmpFolder / app)
    pids = make_ros_graph(app, f'app_{app}', app_path)
    return pids
app.jinja_env.globals.update(show_ros_graph=show_ros_graph)

def read_file(path):
    with open(path,"r") as file:
        content = file.readlines()
    return content

def get_rooms(sid):
    l = rooms(sid)
    l.remove(sid)
    return l

def name_app_room(app):
    return f'Room-{app}'

@app.route('/')
def index():
    return render_template('index.html', AppNames=AppNames)

@app.route('/home')
def home():
    return redirect(url_for('index'))

@app.route('/favicon.ico')
def favicon():
    return 'dummy', 200

@app.route('/app_<string:app_id>')
def app_page(app_id):
    app_folder = Path(TmpFolder, app_id)
    app_folder.mkdir(parents=True, exist_ok=True)
    graph = app_folder / f'{app_id}.svg'
    svg = None
    if graph.is_file():
        svg = open(str(graph)).read()
    return render_template('app.html', appid=app_id, ros_graph = Markup(svg))

CurrentPids = list()
@app.route('/app_<string:app_id>/show_graph')
def show_graph(app_id):
    app_folder = Path(TmpFolder, app_id)
    graph = app_folder / f'{app_id}.svg'
    p_rosgraph = globals()[f"{app_id}"].find_process('rosgraph')

    msg = Message()
    msg.app_id = app_id
    if not p_rosgraph.state:
        socketio.emit('show_graph', msg.dict(), to = name_app_room(app_id))
    show_ros_graph(app_id)
    svg = None
    if graph.is_file():
        svg = open(str(graph)).read()
    return jsonify(result=Markup(svg))

@app.route('/app_<string:app_id>/pause_graph')
def pause_graph(app_id):
    for target in ['rosgraph', 'rosgraph_bridge']:
        p = globals()[f"{app_id}"].find_process(target)
        if p.state:
            stop_command(p.pid)
            p.stop()
    return ("nothing")

"""
Click topic
"""
@app.route('/app_<string:app_id>/topic_<topic_id>/')
def node(app_id, topic_id):
    return redirect(url_for('app_page', app_id=app_id))

"""
Click node
"""
@app.route('/app_<string:app_id>/__<node_id>/')
def topic(app_id, node_id):
    return redirect(url_for('app_page', app_id=app_id))

def monitor_applist_thread():
    """Example of how to send server generated events to clients."""
    count = 0
    while True:
        socketio.sleep(3)
        count += 1
        socketio.emit('show_reception',
                      {'data': AppNames, 'count': count})

@socketio.on('connect')
def on_connect():
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(monitor_applist_thread)
    msg = Message()
    msg.data = f"Client ({request.sid}) connect to DDT Manager"
    session['receive_count'] = session.get('receive_count', 0) + 1
    msg.count = session['receive_count']
    app.logger.info(f"{msg.dict()}")
    socketio.emit('show_log', msg.dict())

"""
regsiter web interface
"""
@socketio.on('register_web')
def register_web(msg):
    global WebID
    WebID = request.sid
    join_room(RoomWebName, sid = WebID)
    RoomWeb.append('web')
    msg = Message()
    msg.data = f"Welcome [Web Interface] ({WebID}) join {get_rooms(WebID)}."
    session['receive_count'] = session.get('receive_count', 0) + 1
    msg.count = session['receive_count']
    print(msg.toJSON())
    socketio.emit('show_log', msg.dict(), to=RoomWebName)

"""
Register a prob per Pod
"""
@socketio.on('register')
def on_join(msg):
    js = Json.loads(msg["data"])
    app_name = js["App"]
    pod_name = js['Pod']
    pod_ip = js['PodIP']
    socket_id = request.sid
    if not (app_name in AppNames):
        # create a list per Room
        globals()[name_app_room(app_name)] = list()
        AppNames.append(app_name)
        # create an instance of Application
        globals()[f"{app_name}"] = Application(app_name)
        for p in Processes:
            globals()[f"{app_name}"].add_process(p)
        # create an instance of Pod
        globals()[f"{pod_name}"] = Pod(pod_name, pod_ip, socket_id)
        global WebID
        join_room(name_app_room(app_name), sid = WebID)
        print(globals()[f"{app_name}"].name)
    join_room(name_app_room(app_name))
    join_room(RoomWebName)
    globals()[name_app_room(app_name)].append(pod_name)
    RoomWeb.append(pod_name)
    app.logger.info(f'Welcome [{pod_name}] ({request.sid}) join {get_rooms(socket_id)}')
    session['receive_count'] = session.get('receive_count', 0) + 1
    msg = Message()
    msg.data = f'Welcome [{pod_name}] ({request.sid}) join {get_rooms(socket_id)}'
    msg.count = session['receive_count']
    socketio.emit('show_log', msg.dict(), to = RoomWebName)

@socketio.on('start_rosgrah')
def register_web(msg):
    m = Message(**msg)
    print(m.processes)
    for process in m.processes:
        pr = Process(**process)
        p = globals()[f"{m.app_id}"].find_process(pr.name)
        if p:
            p.set(pid=pr.pid, state=True)
        else:
            print(f"didn't find process [{pr.name}] in {m.app_id}")

@socketio.event
def my_ping():
    emit('my_pong')

# @socketio.event
# def web_event(message):
#     session['receive_count'] = session.get('receive_count', 0) + 1
#     msg = Message()
#     msg.data = message['data']
#     msg.count = session['receive_count']
#     emit('show_log', msg.dict())

@socketio.on('leave')
def on_leave(data):
    js = Json.loads(data)
    app_name = js["App"]
    pod_name = js['Pod']
    leave_room(app_name)
    app.logger.info(f'{pod_name} has left the room {app_name}, {flask.request.sid}')
    emit(f'leave_{app_name}', pod_name + ' has left the room.', to=app_name)


# @socketio.on('message')
# def on_message(msg):
#     print('received message: ' + msg)
#     endtime = time.time() + 5 # loop for 5 secs
#     while time.time() < endtime:
#         socketio.emit("custom event", f"The time is: {time.strftime('%H:%M:%S')}")
#         socketio.sleep(1)

if __name__=="__main__":

    socketio.run(app)
