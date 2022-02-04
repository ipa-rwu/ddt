import flask
from flask import Flask, render_template, request, session
from flask_socketio import SocketIO, send, join_room, leave_room, emit, rooms
import json
import logging
from threading import Lock

app = Flask(__name__)
from model.ddt_model import Application, Pod

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

class Message:
    def toJSON(self):
        dump =  json.dumps(self, default=lambda o: o.__dict__,
            sort_keys=True, indent=4)
        return dump
    def dict(self):
        return json.loads(self.toJSON())

def show_ros_graph():
    print("clever_function HI")

app.jinja_env.globals.update(show_ros_graph=show_ros_graph)

@app.route('/')
def index():
    return render_template('index.html', AppNames=AppNames)

@app.route('/<appid>')
def app_page(appid):
    return render_template('app.html', appid=appid)

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

# @app.route('/applications')
# def suggestions():
#     text = request.args.get('jsdata')

#     suggestions_list = []

#     if text:
#         r = requests.get('http://suggestqueries.google.com/complete/search?output=toolbar&hl=ru&q={}&gl=in'.format(text))

#         soup = BeautifulSoup(r.content, 'lxml')

#         suggestions = soup.find_all('suggestion')

#         for suggestion in suggestions:
#             suggestions_list.append(suggestion.attrs['data'])

#         #print(suggestions_list)

#     return render_template('index.html', apps=apps)


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

def get_rooms(sid):
    l = rooms(sid)
    l.remove(sid)
    return l

def name_app_room(app):
    return f'Room-{app}'
"""
Register a prob per Pod
"""
@socketio.on('register')
def on_join(msg):
    js = json.loads(msg["data"])
    app_name = js["App"]
    pod_name = js['Pod']
    pod_ip = js['PodIP']
    socket_id = request.sid
    if not (app_name in AppNames):
        # create a list per Room
        globals()[name_app_room(app_name)] = list()
        AppNames.append(app_name)
        # create an istance of Application
        globals()[f"{app_name}"] = Application(app_name)
        # create an istance of Pod
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
    js = json.loads(data)
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
