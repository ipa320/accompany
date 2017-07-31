Function.prototype.bind = function(scope) {
  var _function = this;
  
  return function() {
    return _function.apply(scope, arguments);
  }
}

function uiHelper() {
}

uiHelper.prototype = {
	load : function(img) {
		this.autoRefresh(img);
	},

	autoRefresh : function(img) {
		var self = this;
		setTimeout(function() {
			$(img).attr("src", "image");
			//doing this causes a massive memory leak
			//since the image is an svg, it should be fine without cache busting
			//$(img).attr("src", "image?cachebust=" + new Date().getTime());
			self.autoRefresh(img);
		}, 2000);
	},
}
