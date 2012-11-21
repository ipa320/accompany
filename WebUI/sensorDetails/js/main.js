Function.prototype.bind = function(scope) {
  var _function = this;
  
  return function() {
    return _function.apply(scope, arguments);
  }
}

function dataHelper() {
}

dataHelper.prototype = {
	getResponses : function() {
		var responses = null;
		$.ajax({
			url : 'data',
			dataType : 'json',
			async : false,
			timeout: 3000,
			success : function(data, textStatus, jqXHR) {
				responses = data
			},
		});

		return responses;
	},

	pollResponses : function(callback, root) {
		var self = this;
		var data = self.getResponses();
		callback(root, data);
		setTimeout(function() {
			self.pollResponses(callback, root);
		}, 2000);
	},
}

function uiHelper() {
	
}

uiHelper.prototype = {
	load : function(targetDiv) {
		var dao = new dataHelper();
		dao.pollResponses(this.fill.bind(this), targetDiv);
	},

	fill : function(root, lastId, data) {
		$(root).empty();
		
		var table = $('<table></table>');
		var head = $('<thead></thead>');
		var row = $('<tr></tr>');
		var col = $('<th></th>');
		$(col).text = 'Room';
		$(row).append(col);
		var col = $('<th></th>');
		$(col).text = 'Channel';
		$(row).append(col);
		var col = $('<th></th>');
		$(col).text = 'Value';
		$(row).append(col);
		var col = $('<th></th>');
		$(col).text = 'Status';
		$(row).append(col);
		$(head).append(row);
		$(table).append(head);
		var body = $('<tbody></tbody');
        for (channel in data){
        	row = $('<tr></tr>')
        	$(row).prop('style', 'background-color:' + channel['color']);
	    	var data = $('<td></td>')
	    	$(data).text = channel['room'];
	    	$(row).append(data);
	    	
	    	data = $('<td></td>')
	    	$(data).text = channel['channel'];
	    	$(row).append(data);
	    	
	    	data = $('<td></td>')
	    	$(data).text = channel['value'];
	    	$(row).append(data);
	    	
	    	data = $('<td></td>')
	    	$(data).text = channel['status'];
	    	$(row).append(data);
        }
	},
}