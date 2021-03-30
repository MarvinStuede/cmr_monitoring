OccupancyGrid = function(options) {
  options = options || {};
  var message = options.message;
  // internal drawing canvas
  var canvas = document.createElement('canvas');
  var context = canvas.getContext('2d');
  // save the metadata we need
  this.pose = new ROSLIB.Pose({
    position : message.info.origin.position,
    orientation : message.info.origin.orientation
  });
  // set the size
  this.width = message.info.width;
  this.height = message.info.height;
  canvas.width = this.width;
  canvas.height = this.height;
  var imageData = context.createImageData(this.width, this.height);
  for ( var row = 0; row < this.height; row++) {
    for ( var col = 0; col < this.width; col++) {
      // determine the index into the map data
      var mapI = col + ((this.height - row - 1) * this.width);
      // determine the value
      var data = message.data[mapI];
      var val;
      if (data >= 100) {
        val = 255;
      } else if (data <= 0) {
        val = 0;
      } else {
        val = data/100*255;
      }
      // determine the index into the image data array
      var i = (col + (row * this.width)) * 4;
      
      if (val >= 253) {
          
      // r
      imageData.data[i] = 0;
      // g
      imageData.data[++i] = 255;
      // b
      imageData.data[++i] = 255;
      // a
      imageData.data[++i] = 255;
      
      }

      else if (val === 0){
      // r
      imageData.data[i] = 0;
      // g
      imageData.data[++i] = 0;
      // b
      imageData.data[++i] = 0;
      // a
      imageData.data[++i] = 0;
     
      }
      else{
      // r
      imageData.data[i] = val;
      // g
      imageData.data[++i] = 0;
      // b
      imageData.data[++i] = 255-val;
      // a
      imageData.data[++i] = 255;
      }
      }
  }
  context.putImageData(imageData, 0, 0);
  // create the bitmap
  createjs.Bitmap.call(this, canvas);
  // change Y direction
  this.y = -this.height * message.info.resolution;
  
  // scale the image
  this.scaleX = message.info.resolution;
  this.scaleY = message.info.resolution;
  this.width *= this.scaleX;
  this.height *= this.scaleY;
  // set the pose
  this.x += this.pose.position.x;
  this.y -= this.pose.position.y;
};
OccupancyGrid.prototype.__proto__ = createjs.Bitmap.prototype;

OccupancyGridClient = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var topic = options.topic || '/map';
  this.continuous = options.continuous;
  this.rootObject = options.rootObject || new createjs.Container();
  // current grid that is displayed
  // create an empty shape to start with, so that the order remains correct.
  this.currentGrid = new createjs.Shape();
  this.rootObject.addChild(this.currentGrid);
  // work-around for a bug in easeljs -- needs a second object to render correctly
  this.rootObject.addChild(new ROS2D.Grid({size:1}));
  // subscribe to the topic
  var rosTopic = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : 'nav_msgs/OccupancyGrid',
    compression : 'png'
  });
  rosTopic.subscribe(function(message) {
    // check for an old map
    var index = null;
    if (that.currentGrid) {
      index = that.rootObject.getChildIndex(that.currentGrid);
      that.rootObject.removeChild(that.currentGrid);
    }
    that.currentGrid = new OccupancyGrid({
      message : message
    });
    if (index !== null) {
      that.rootObject.addChildAt(that.currentGrid, index);
    }
    else {
      that.rootObject.addChild(that.currentGrid);
    }
    that.emit('change');
    // check if we should unsubscribe
    if (!that.continuous) {
      rosTopic.unsubscribe();
    }
  });
};
OccupancyGridClient.prototype.__proto__ = EventEmitter2.prototype;

