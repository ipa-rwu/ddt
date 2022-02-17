from flask import Flask, render_template, request, session, redirect, url_for, jsonify
from flask_socketio import SocketIO, send, join_room, leave_room, emit, rooms
import logging
from threading import Lock
from pprint import pprint

from ddt_utils.model import Message
from ddt_utils.model import Pod
from ddt_utils.model import Process

from ddt_utils.utils import ProcessList

from ddt_manager.utils import AppNames, RoomWebName, RoomWeb, WebID
from ddt_manager.utils import name_select_nodes
from ddt_manager.utils import get_app_rosgraph_path
from ddt_manager.utils import name_app_obj
from ddt_manager.utils import name_app_room
from ddt_manager.utils import get_list
from ddt_manager.utils import get_rooms
from ddt_manager.utils import cleanup_folder

from ddt_manager.manager.action import update_ros_graph
from ddt_manager.manager.action import update_rosmodels
from ddt_manager.manager.action import combine_rosgraphs
from ddt_manager.manager.action import show_svg
from ddt_manager.model import Application, DebugElement

app = Flask("ddt_manager")
app.config['SECRET_KEY'] = 'secret'
app.config['DEBUG'] = True

socketio = SocketIO(app)
thread = None
thread_lock = Lock()
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

@app.route('/')
def index():
    return render_template('index.html', AppNames=AppNames)

@app.route('/home')
def home():
    return redirect(url_for('index'))

@app.route('/favicon.ico')
def favicon():
    return 'dummy', 200

@app.route('/app_<string:app_id>', methods=["POST","GET"])
def app_page(app_id):
    res= ""
    app_graph_path = get_app_rosgraph_path(app_id)
    debug_list = globals()[name_select_nodes(app_id)]

    if app_graph_path.is_file():
        res = show_svg(app_graph_path)
    button_show_graph = True
    try:
        app_model = globals()[name_app_obj(app_id)]
        for pod, proc in app_model.find_processes(ProcessList.RosGraphProcess.name):
            if proc.state:
                button_show_graph = False
    except KeyError:
        app.logger.info(f"Model: {name_app_obj(app_id)} doesn't register yet")
    return render_template('app.html', app_id=app_id,
                                        ros_graph = res,
                                        debug_list = debug_list,
                                        button_show_graph = button_show_graph)


@app.route('/app_<string:app_id>/show_graph')
def show_graph(app_id):
    app_model = globals()[name_app_obj(app_id)]

    for pod in app_model.pods:
        pod_id = pod.name
        pod_ros_graph_process = pod.find_process(ProcessList.RosGraphProcess.name)
        msg = Message()
        msg.app_id = app_id
        msg.pod_id = pod_id
        if not pod_ros_graph_process.state:
            # if app_graph_path.is_file():
            #     return jsonify(result=show_svg(app_graph_path))
            app.logger.info(f'Trigger pod [{pod_id} start "ROS Graph maker"]')
            socketio.emit('show_graph', msg.dict(), to = pod.socket_id)
        pod_rosmodel_process = pod.find_process(ProcessList.NodeParserProcess.name)
        if not pod_rosmodel_process.state:
            # if app_graph_path.is_file():
            #     return jsonify(result=show_svg(app_graph_path))
            app.logger.info(f'Trigger pod [{pod_id} start "ROS Node Parser"]')
            socketio.emit('get_node_model', msg.dict(), to = pod.socket_id)
        update_ros_graph(app_id, pod_id)
        app_model.find_pod(pod_id).nodes = list()
        app_model.add_nodes(pod_id, update_rosmodels(app_id, pod_id))
    app.logger.info(f'Application {app_id} status: ')
    pprint(globals()[name_app_obj(app_id)].dict())
    res = combine_rosgraphs(app_obj=app_model)
    return jsonify(result=res)

def stop_process(app_id):
    for target in [ProcessList.RosGraphProcess.name, ProcessList.RosGraphProcess.name, ProcessList.NodeParserProcess.name]:
        for pod, process in globals()[name_app_obj(app_id)].find_processes(target):
            msg = Message()
            setattr(msg, pod.name, process.pid)
            # app.logger.info(f"Stop process in {app_id}: {process.name} from {pod.name}")
            if process.state:
                socketio.emit('pause_graph', msg.dict(), to = name_app_room(app_id))
                process.stop()

@app.route('/app_<string:app_id>/pause_graph')
def pause_graph(app_id):
    stop_process(app_id)
    return ("nothing")

"""
Click topic
"""
@app.route('/app_<string:app_id>/<pod_id>/topic_<topic_id>/add')
def add_topic(app_id, pod_id, topic_id):
    stop_process(app_id)
    return redirect(url_for('app_page', app_id=app_id))


"""
ignore rosgraph node
"""
@app.route('/app_<string:app_id>/<pod_id>/__rosgraph_creator/add', methods=['GET', 'POST'])
def ignore_node(app_id, pod_id):
    stop_process(app_id)
    return redirect(url_for('app_page', app_id=app_id))

"""
add debug node
"""
@app.route('/app_<string:app_id>/<pod_id>/__<node_id>/add', methods=['GET', 'POST'])
def add_node(app_id, pod_id, node_id):
    stop_process(app_id)
    app.logger.info(f'Select debugging node [{node_id}] from pod [{pod_id}]')
    debug_instance = DebugElement(pod=pod_id, node=node_id)
    # return jsonify(result=debug_instance.json())
    try:
        debug_list = globals()[name_select_nodes(app_id)]
        if not debug_instance in debug_list:
            debug_list.append(debug_instance)
    except KeyError:
        app.logger.error(f'debug list {app_id} is not registered yet')
    return redirect(url_for('app_page', app_id=app_id))

"""
delete node
"""
@app.route('/app_<string:app_id>/<pod_id>/__<node_id>/delete', methods=['GET', 'POST'])
def delete_debug_node(app_id, pod_id, node_id):
    app.logger.info(f'Deleting debugging node [{node_id}] from pod [{pod_id}]')
    delete_debug_instance = DebugElement(pod=pod_id, node=node_id)
    # return jsonify(result=debug_instance.json())
    try:
        debug_list = globals()[name_select_nodes(app_id)]
        if  delete_debug_instance in debug_list:
            debug_list.remove(delete_debug_instance)
    except KeyError:
        app.logger.error(f'debug list {app_id} is not registered yet')
    return redirect(url_for('app_page', app_id=app_id))

"""
handle debug list
"""
@app.route('/app_<string:app_id>/debug', methods=['GET','POST'])
def handle_debug(app_id):
    if request.method == 'POST':
        for l in get_list(('debug_pod', 'debug_node')):
            ele = DebugElement(node=l['debug_node'], pod=l['debug_pod'])
            globals()[name_app_obj(app_id)].debug.append(ele)
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
    socketio.emit('show_log', msg.dict(), to=RoomWebName)

"""
Register a prob per Pod
"""
@socketio.on('register')
def on_join(msg):
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    pod_ip = m.pod_ip
    domain_id = m.domain_id
    socket_id = request.sid
    if name_app_obj(app_id) not in globals():
        # create a list per Room
        globals()[name_app_room(app_id)] = list()
        AppNames.append(app_id)
        # create an instance of Application
        globals()[name_app_obj(app_id)] = Application(name = app_id)
        globals()[name_select_nodes(app_id)] = list()
        if WebID:
            join_room(name_app_room(app_id), sid = WebID)
        RoomWeb.append(app_id)
        app.logger.info(f'Create a new application: {app_id}')

    if not (pod_id in globals()[name_app_obj(app_id)].all_pods()):
        # create an instance of Pod
        def get_process():
            for name in ProcessList.list():
                yield Process(name=name)
        new_pod = Pod(name = pod_id,
                    ip = pod_ip,
                    domain_id = int(domain_id) ,
                    socket_id = socket_id,
                    processes = list(get_process()))
        app.logger.info(f'new pod is: ')
        pprint(new_pod.dict())
        globals()[name_app_obj(app_id)].add_pod(Pod(name = pod_id,
                                                    ip = pod_ip,
                                                    domain_id = int(domain_id) ,
                                                    socket_id = socket_id,
                                                    processes = list(get_process())))
        # pod join Room_app_{app_id}
        join_room(name_app_room(app_id), sid = socket_id)
        # app join Room_web
        join_room(RoomWebName, sid = socket_id)
        globals()[name_app_room(app_id)].append(pod_id)
        RoomWeb.append(pod_id)
        msg = Message()
        msg.data = f'Welcome [{pod_id}] ({request.sid}) join {get_rooms(socket_id)}, {globals()[name_app_room(app_id)]} are in the {name_app_room(app_id)}'
        app.logger.info(msg.data)
        msg.count = session['receive_count']
        session['receive_count'] = session.get('receive_count', 0) + 1
        socketio.emit('show_log', msg.dict(), to = RoomWebName)

@socketio.on('started_rosgrah')
def started_rosgrah(msg):
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    for process in m.processes:
        print(process)
        p = globals()[name_app_obj(app_id)].find_pod(pod_id).find_process(process['name'])
        if p:
            p.start(pid=process['pid'])
            app.logger.info(f"Application {app_id}: {pod_id} started [{p.name} ({process['pid']})]")
        else:
            app.logger.error(f"Application {app_id}: {pod_id} didn't have process [{p.name}]")

@socketio.on('started_rosmodel_parser')
def started_rosmodel_parser(msg):
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    process_name = ProcessList.NodeParserProcess.name
    pid = m.pid
    p = globals()[name_app_obj(app_id)].find_pod(pod_id).find_process(process_name)
    if p:
        p.start(pid=pid)
        app.logger.info(f"Application {app_id}: {pod_id} started [{process_name}] ({pid})]")
    else:
        app.logger.error(f"Application {app_id}: {pod_id} didn't have process [{process_name}]")

# @socketio.event
# def my_ping():
#     emit('my_pong')

@socketio.on('leave')
def on_leave(msg):
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    app_model = globals()[name_app_obj(app_id)]
    p = app_model.find_pod(pod_id)
    leave_room(name_app_room(app_id), sid=p.socket_id)
    leave_room(RoomWebName,  sid=p.socket_id)
    # remove from the list of a room
    room_list = globals()[name_app_room(app_id)]
    room_list.remove(pod_id)
    msg = Message()
    msg.data = f'{pod_id} from [{m.app_id}] has left the [{name_app_room(m.app_id)}], {room_list} are still in the room.'
    app.logger.info(msg.data)
    # remove from model
    app_model.remove_pod(p)
    # remove pod folder
    cleanup_folder(app_id, pod_id)
    RoomWeb.remove(pod_id)
    # no pods left, remove app
    if not len(room_list):
        cleanup_folder(app_id)
        del globals()[name_app_obj(app_id)]
        del globals()[name_app_room(app_id)]
        del globals()[name_select_nodes(app_id)]
        RoomWeb.remove(app_id)
        AppNames.remove(app_id)
    socketio.emit('show_log', msg.dict(), to = RoomWebName)

if __name__=="__main__":

    socketio.run(app)
