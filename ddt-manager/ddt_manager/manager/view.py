from pathlib import Path
from ddt_utils.actions import start_command
from flask import Flask, render_template, request, session, redirect, url_for, jsonify
from flask import flash
from werkzeug.utils import secure_filename
from flask_socketio import SocketIO, join_room, leave_room
import logging
import logging.handlers
from threading import Lock
import re

from ddt_utils.model import Message
from ddt_utils.model import Pod
from ddt_utils.model import Process

from ddt_utils.utils import ProcessList
from ddt_utils.utils import SocketActionList
from ddt_utils.utils import TmpFolder
from ddt_utils.utils import DebugPodPrefix
from ddt_utils.utils import ShowRosGraphProcesses

from ddt_manager.utils import ALLOWED_EXTENSIONS, AppNames, RoomWebName, RoomWeb, WebID
from ddt_manager.utils import name_debug_nodes_list
from ddt_manager.utils import get_app_rosgraph_path
from ddt_manager.utils import name_app_obj
from ddt_manager.utils import name_app_room
from ddt_manager.utils import get_list
from ddt_manager.utils import get_rooms
from ddt_manager.utils import cleanup_folder
from ddt_manager.utils import get_log_path
from ddt_manager.utils import debug_deployment_folder

from ddt_manager.manager.action import update_ros_graph
from ddt_manager.manager.action import combine_rosgraphs
from ddt_manager.manager.action import show_svg
from ddt_manager.manager.action import check_state_emit
from ddt_manager.manager.action import set_processes_group
from ddt_manager.manager.action import update_app_model

from ddt_manager.model import Application, DebugElement

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret'
app.config['DEBUG'] = True
app.config['UPLOAD_FOLDER'] = TmpFolder

socketio = SocketIO(app)

thread = None
thread_lock = Lock()
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)
handler = logging.handlers.RotatingFileHandler(
        get_log_path(),
        maxBytes=1024 * 1024)
# app.logger.addHandler(handler)

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
    debug_list = globals()[name_debug_nodes_list(app_id)]

    if app_graph_path.is_file():
        res = show_svg(app_graph_path)
    button_show_graph = True
    p = Path(debug_deployment_folder(app_id=app_id)).glob('**/*')
    # print(list(p))
    files = [x.name for x in p if x.is_file()]
    # print(files)
    try:
        app_model = globals()[name_app_obj(app_id)]
        for pod, proc in app_model.find_processes(ProcessList.RosGraphProcess.name):
            if proc.state:
                button_show_graph = False
    except KeyError:
        app.logger.warn(f"Model: {name_app_obj(app_id)} doesn't register yet")
    return render_template('app.html', app_id=app_id,
                                        ros_graph = res,
                                        debug_list = debug_list,
                                        button_show_graph = button_show_graph,
                                        upload_files = files)

@app.route('/app_<string:app_id>/show_graph')
def show_graph(app_id):
    app_model = globals()[name_app_obj(app_id)]

    for pod in app_model.pods:
        pod_id = pod.name
        msg = Message()
        msg.app_id = app_id
        msg.pod_id = pod_id

        check_state_emit(pod, ProcessList.RosGraphProcess, msg.dict(), socketio, pod.socket_id)

        check_state_emit(pod, ProcessList.NodeParserProcess, msg.dict(), socketio, pod.socket_id)

        check_state_emit(pod, ProcessList.LifeCycleParserProcess, msg.dict(), socketio, pod.socket_id)

        update_ros_graph(app_id, pod_id, pod_ip = pod.ip, logger=app.logger)

    res = combine_rosgraphs(app_obj=app_model)
    return jsonify(result=res)

def stop_process(app_id):
    app_model = globals()[name_app_obj(app_id)]
    for pod in app_model.pods:
        msg = Message()
        msg.pod = pod.name
        socketio.emit(SocketActionList.PauseGraph.value, msg.dict(), to = pod.socket_id) #to = name_app_room(app_id)
        processes = [{"name" : p, "pid" : None} for p in ShowRosGraphProcesses]
        set_processes_group(pod, processes, app.logger, stop=True)
    app.logger.info(f'show_graph: Application {app_id} status: ')
    app.logger.info(globals()[name_app_obj(app_id)].dict())

@app.route('/app_<string:app_id>/pause_graph')
def pause_graph(app_id):
    stop_process(app_id)
    app_model = globals()[name_app_obj(app_id)]
    update_app_model(app_model, logger=app.logger)
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
@app.route('/app_<string:app_id>/<pod_id>/__$rosgraph_creator/add', methods=['GET', 'POST'])
def ignore_node(app_id, pod_id):
    stop_process(app_id)
    return redirect(url_for('app_page', app_id=app_id))

"""
add debug node
"""
@app.route('/app_<string:app_id>/<pod_id>/__<node_id>/add', methods=['GET', 'POST'])
def add_node(app_id, pod_id, node_id):
    stop_process(app_id)
    node_id = node_id.replace('$', '/')

    app.logger.info(f'Select debugging node [{node_id}] from pod [{pod_id}]')
    debug_instance = DebugElement(pod=pod_id, node=node_id)
    # return jsonify(result=debug_instance.json())
    try:
        debug_list = globals()[name_debug_nodes_list(app_id)]
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
    node_id = node_id.replace('$', '/')
    app.logger.info(f'Deleting debugging node [{node_id}] from pod [{pod_id}]')
    delete_debug_instance = DebugElement(pod=pod_id, node=node_id)
    # return jsonify(result=debug_instance.json())
    try:
        debug_list = globals()[name_debug_nodes_list(app_id)]
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
    app_model = globals()[name_app_obj(app_id)]
    if request.method == 'POST':
        # update_app_model(app_model, logger=app.logger)
        pod_nodes = dict()
        for l in get_list(('debug_pod', 'debug_node')):
            debug_node_name = l['debug_node']
            debug_pod_name = l['debug_pod']
            if not debug_pod_name in pod_nodes.keys():
                pod_nodes[debug_pod_name] = list()
            if not debug_node_name in pod_nodes[debug_pod_name]:
                pod_nodes[debug_pod_name].append(debug_node_name)
            ele = DebugElement(node=debug_node_name, pod=debug_pod_name)
            app_model.debug.append(ele)
        for pod_id in pod_nodes.keys():
            pod_model = app_model.find_pod(pod_id)
            msg = Message()
            msg.nodes = pod_nodes[pod_id]
            msg.app_id = app_id
            msg.pod_id = pod_id
            socketio.emit(SocketActionList.Debug.value, msg.dict(), to = pod_model.socket_id)
        app.logger.info(f'Request debugging in Application {app_id}: ')
        app.logger.info(globals()[name_app_obj(app_id)].debug)
    return redirect(url_for('app_page', app_id=app_id))

def allowed_file(filename):
    return '.' in filename and \
           filename.rsplit('.', 1)[1].lower() in ALLOWED_EXTENSIONS

@app.route('/app_<string:app_id>/upload', methods=['GET', 'POST'])
def upload(app_id):
    if request.method == 'POST':
        # check if the post request has the file part
        if 'file' not in request.files:
            flash('No file part')
            return redirect(request.url)
        file = request.files['file']
        # If the user does not select a file, the browser submits an
        # empty file without a filename.
        if not re.match(DebugPodPrefix, file.filename):
            flash('Plese name deployment file with prefix[{DebugPodPrefix}]')
            return redirect(url_for('app_page', app_id=app_id))
        if file.filename == '':
            flash('No selected file')
            return redirect(url_for('app_page', app_id=app_id))
        if file and allowed_file(file.filename):
            filename = secure_filename(file.filename)
            file.save(Path(debug_deployment_folder(app_id=app_id)/filename))
            return redirect(url_for('app_page', app_id=app_id))

@app.route('/app_<string:app_id>/delete_deployments', methods=['GET', 'POST'])
def delete_deployments(app_id):
    app.logger.info(f'delete deployment')
    if request.method == 'POST':
        filename = request.form.get("file")
        Path(debug_deployment_folder(app_id=app_id)/filename).unlink()
        return redirect(url_for('app_page', app_id=app_id))

@app.route('/app_<string:app_id>/comfirm_upload', methods=['GET', 'POST'])
def comfirm_upload(app_id):
    if request.method == 'POST':
        debug_deployment_files = get_list(["file"])
        app.logger.info(f'Get deployment files for debug target nodes: {debug_deployment_files}')
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
    new_flg = False
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    pod_ip = m.pod_ip
    domain_id = m.domain_id
    socket_id = request.sid
    def _create_model():
        if name_app_obj(app_id) not in globals():
            # create a list per Room
            globals()[name_app_room(app_id)] = list()
            AppNames.append(app_id)
            # create an instance of Application
            globals()[name_app_obj(app_id)] = Application(name = app_id)
            globals()[name_debug_nodes_list(app_id)] = list()
            if WebID:
                join_room(name_app_room(app_id), sid = WebID)
            RoomWeb.append(app_id)
            app.logger.info(f'Create a new application: {app_id}')

    _create_model()
    if pod_id in globals()[name_app_obj(app_id)].all_pods():
        app_model = globals()[name_app_obj(app_id)]
        pod_model = app_model.find_pod(pod_id)
        if pod_model.socket_id != socket_id:
            remove_pod(app_model=app_model, pod_model=pod_model)
            _create_model()
    if not(pod_id in globals()[name_app_obj(app_id)].all_pods()):
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
        app.logger.info(new_pod.dict())
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

    if re.match(DebugPodPrefix, pod_id):
        socketio.emit(ProcessList.DebugBridgeProcess.value, msg.dict(), to = socket_id)

# muti processes
@socketio.on(f'started_{ProcessList.LifeCycleParserProcess.value}')
def started_rosgrah(msg):
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    p = globals()[name_app_obj(app_id)].find_pod(pod_id)
    set_processes_group(p, m.processes, app.logger, start=True)

@socketio.on(f'started_{ProcessList.RosGraphProcess.value}')
def started_rosgrah(msg):
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    p = globals()[name_app_obj(app_id)].find_pod(pod_id)
    set_processes_group(p, m.processes, app.logger, start=True)

@socketio.on(f'started_{ProcessList.NodeParserProcess.value}')
def started_rosmodel_parser(msg):
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    p = globals()[name_app_obj(app_id)].find_pod(pod_id)
    set_processes_group(p, m.processes, app.logger, start=True)

@socketio.on(f'started_{ProcessList.DebugBridgeProcess.value}')
def started_ori_bridge(msg):
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    p = globals()[name_app_obj(app_id)].find_pod(pod_id)
    set_processes_group(p, m.processes, app.logger, start=True)
    # deploy debug pod
    debug_list = globals()[name_debug_nodes_list(app_id)]
    # for debug_node in debug_list:
    #     start_call()


# @socketio.event
# def my_ping():
#     emit('my_pong')

@socketio.on('leave')
def on_leave(msg):
    m = Message(**msg)
    app_id = m.app_id
    pod_id = m.pod_id
    app_model = globals()[name_app_obj(app_id)]
    pod_model = app_model.find_pod(pod_id)
    remove_pod(app_model=app_model, pod_model=pod_model)
    room_list = globals()[name_app_room(app_model.name)]
    msg = Message()
    msg.data = f'{pod_id} from [{m.app_id}] has left the [{name_app_room(m.app_id)}], {room_list} are still in the room.'
    app.logger.info(msg.data)
    socketio.emit('show_log', msg.dict(), to = RoomWebName)

def remove_pod(app_model, pod_model):
    leave_room(name_app_room(app_model.name), sid=pod_model.socket_id)
    leave_room(RoomWebName,  sid=pod_model.socket_id)

    room_list = globals()[name_app_room(app_model.name)]
    room_list.remove(pod_model.name)

    app_model.remove_pod(pod_model)
    # remove pod folder
    cleanup_folder(app_model.name, pod_model.name)
    RoomWeb.remove(pod_model.name)
    # no pods left, remove app
    if not len(room_list):
        cleanup_folder(app_model.name)
        del globals()[name_app_obj(app_model.name)]
        del globals()[name_app_room(app_model.name)]
        del globals()[name_debug_nodes_list(app_model.name)]
        RoomWeb.remove(app_model.name)
        AppNames.remove(app_model.name)

# if __name__=="__main__":

#     socketio.run(app)
