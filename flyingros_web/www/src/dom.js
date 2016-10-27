// dom

var modal = {};
var mission = {};

modal.options = {
  valueNames: ['index', 'type', 'name', 'target', 'data', 'ID', 'bonus']
   // item : '<tr><td class="index"></td><td class="type"></td><td class="name"></td><td class="target"></td></tr>',
 };

function SelectText(el, win) {
    win = win || window;
    var doc = win.document, sel, range;
    if (win.getSelection && doc.createRange) {
        sel = win.getSelection();
        range = doc.createRange();
        range.selectNodeContents(el);
        sel.removeAllRanges();
        sel.addRange(range);
    } else if (doc.body.createTextRange) {
        range = doc.body.createTextRange();
        range.moveToElementText(el);
        range.select();
    }
}

function syntaxHighlight(json) {
    if (typeof json != 'string') {
         json = JSON.stringify(json, undefined, 2);
    }
    json = json.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/\n/g, '<br>').replace(/\ /g, '&nbsp;&nbsp;');;
    return json.replace(/("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)/g, function (match) {
        var cls = 'number';
        var br = '';
        if (/^"/.test(match)) {
            if (/:$/.test(match)) {
                cls = 'key';
            } else {
                cls = 'string';
            }
        } else if (/true|false/.test(match)) {
            cls = 'boolean';
        } else if (/null/.test(match)) {
            cls = 'null';
        }
        return '<span class="' + cls + '">' + match + '</span>';
    });
}

function show_mission_json(mission_json){
  mission.remove();
  var l = mission_json.tasks.length;
  var missionArray = [];
  for (var i = 0; i < l; i++) {
    missionArray.push(taskHelper.toList(mission_json.tasks[i], i));
  }
  mission.add(missionArray);
}

 function dom_init() {
  mission = new List('Mission', modal.options);

  document.getElementById("getMission")
  .addEventListener("click", function(){
    cmd.mission.get.callService({}, function(result){
      show_mission_json(result.mission);
    });
  });

  document.getElementById("removeMission").addEventListener("click", function(){
   cmd.mission.reset();
 });

  document.getElementById("selectTextExport").addEventListener("click", function(){
   SelectText(modal.exportMission.modal.querySelector('.exportData'), window);
 });

  modal.addTask = new Modalise('addTaskModal', {
    btnsOpen : [document.getElementById('addTask')]
  }).attach();

  modal.exportMission = new Modalise('exportModal', {
    btnsOpen : [document.getElementById('exportMission')]
  }).attach().on('onShow', function(){
    var textareajson = this.querySelector('.exportData');
    cmd.mission.get.callService({}, function(result){
      textareajson.innerHTML = syntaxHighlight(JSON.stringify(result.mission, null, 2));
    });
  });

  modal.importMission = new Modalise('importModal', {
    btnsOpen : [document.getElementById('importMission')]
  }).attach().on('onShow', function(){
    this.querySelector('textarea').innerHTML = "";
  }).on('onConfirm', function(){
    var json_mission = JSON.parse(this.querySelector('textarea').value);

    cmd.mission.reset(function(){
      cmd.mission.add.callService({mission: json_mission} ,function(result){
        show_mission_json(json_mission);
      });
    });
  
  })

  modal.removeTask = new Modalise('removeModal', {
  }).attach().on('onConfirm', function(event){
    id = Number(modal.removeTask.modal.querySelector('.ID').innerHTML);

    if(!isFinite(id)) return false;

    cmd.task.remove.callService({task: {
      ID : parseInt(id)
    }}, function(result){
      mission.remove('ID', id);
    });
  });

  modal.removeTask.showData = function(task){
    modal.removeTask.modal.querySelector('.name').innerHTML = task._values.name
    modal.removeTask.modal.querySelector('.index').innerHTML = task._values.index
    modal.removeTask.modal.querySelector('.type').innerHTML = task._values.type
    modal.removeTask.modal.querySelector('.bonus').innerHTML = task._values.bonus
    modal.removeTask.modal.querySelector('.target').innerHTML = task._values.target
    modal.removeTask.modal.querySelector('.ID').innerHTML = task._values.ID
    modal.removeTask.modal.style.display = "block";
  };

  document.getElementById("clickTbody").addEventListener('click', function(e){
    task = getClicked(event);
    modal.removeTask.showData(task)
  });
}

var getClicked = function (event) {
  var tr = event.srcElement.closest('tr');
  var a = tr.getElementsByClassName('ID');
  if (a.length == 1) {
    var b = mission.get('ID', a[0].innerText)
    if (b.length == 1) {
      // save data to use
      dynamicData.currentObject = b[0];
      return b[0];
    }
  }
// Failed
return false;
};
