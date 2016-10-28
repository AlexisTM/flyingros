// form_validator

var form_validator_init = function(){
   var _m = modal.addTask.modal;
  _m.querySelector('.type').addEventListener('change', validator);
};

var validator = function(e){
    var mission_type = taskHelper.getMissionFromSelect(e.srcElement);
    var _m = modal.addTask.modal;
    switch(mission_type){
      case taskHelper.type.TARGET : {
        _m.querySelector('.targetx').required = true;
        _m.querySelector('.targety').required = true;
        _m.querySelector('.targetz').required = true;
        _m.querySelector('.name').required = false;
        _m.querySelector('.data').required = true;
        _m.querySelector('.yaw').required = true;
        break;
      };

      case taskHelper.type.INIT_UAV : {
        _m.querySelector('.targetx').required = false;
        _m.querySelector('.targety').required = false;
        _m.querySelector('.targetz').required = false;
        _m.querySelector('.name').required = false;
        _m.querySelector('.data').required = true;
        _m.querySelector('.yaw').required = false;
        break;
      };

      case taskHelper.type.ARM : {
        _m.querySelector('.targetx').required = false;
        _m.querySelector('.targety').required = false;
        _m.querySelector('.targetz').required = false;
        _m.querySelector('.name').required = false;
        _m.querySelector('.data').required = true;
        _m.querySelector('.yaw').required = false;
        break;
      };

      case taskHelper.type.DISARM : {
        _m.querySelector('.targetx').required = false;
        _m.querySelector('.targety').required = false;
        _m.querySelector('.targetz').required = false;
        _m.querySelector('.name').required = false;
        _m.querySelector('.data').required = true;
        _m.querySelector('.yaw').required = false;
        break;
      };

      case taskHelper.type.LOITER : {
        _m.querySelector('.targetx').required = false;
        _m.querySelector('.targety').required = false;
        _m.querySelector('.targetz').required = false;
        _m.querySelector('.name').required = false;
        _m.querySelector('.data').required = true;
        _m.querySelector('.yaw').required = false;
        break;
      };


      case taskHelper.type.TAKEOFF : {
        _m.querySelector('.targetx').required = false;
        _m.querySelector('.targety').required = false;
        _m.querySelector('.targetz').required = false;
        _m.querySelector('.name').required = false;
        _m.querySelector('.data').required = true;
        _m.querySelector('.yaw').required = false;
        break;
      };

      case taskHelper.type.LAND : {
        _m.querySelector('.targetx').required = false;
        _m.querySelector('.targety').required = false;
        _m.querySelector('.targetz').required = false;
        _m.querySelector('.name').required = false;
        _m.querySelector('.data').required = true;
        _m.querySelector('.yaw').required = false;
        break;
      };

      case taskHelper.type.GRAB : {
        _m.querySelector('.targetx').required = false;
        _m.querySelector('.targety').required = false;
        _m.querySelector('.targetz').required = false;
        _m.querySelector('.name').required = false;
        _m.querySelector('.data').required = true;
        _m.querySelector('.yaw').required = false;
        break;
      };

      case taskHelper.type.TEST : {
        _m.querySelector('.targetx').required = false;
        _m.querySelector('.targety').required = false;
        _m.querySelector('.targetz').required = false;
        _m.querySelector('.name').required = false;
        _m.querySelector('.data').required = false;
        _m.querySelector('.yaw').required = false;
        break;
      };
    }
};