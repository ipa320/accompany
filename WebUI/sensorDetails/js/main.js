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

	fill : function(root, data) {
		$(root).empty();
		
		var table = $('<table></table>');
        $(root).append(table);
        
		var head = $('<thead></thead>');
		$(table).append(head);
		
		var row = $('<tr></tr>');
		$(head).append(row);
		
		var col = $('<th></th>');
		$(row).append(col);
		$(col).text('Room');
		
		var col = $('<th></th>');
		$(row).append(col);
		$(col).text('Channel');
		
		var col = $('<th></th>');
		$(row).append(col);
		$(col).text('Value');
		
		var col = $('<th></th>');
		$(row).append(col);
		$(col).text('Status');
		
		var body = $('<tbody></tbody');
		$(table).append(body);
		
        for (key in data){
        	var channel = data[key];
        	var row = $('<tr></tr>')
        	$(body).append(row);        	
        	$(row).css('background-color', channel['color']);
        	
	    	var cell = $('<td></td>')
	    	$(row).append(cell);
	    	$(cell).text(channel['room']);
	    	
	    	cell = $('<td></td>')
	    	$(row).append(cell);
	    	$(cell).text(channel['channel']);
	    	
	    	cell = $('<td></td>')
	    	$(row).append(cell);
	    	$(cell).text(channel['value']);
	    	
	    	cell = $('<td></td>')
	    	$(row).append(cell);
	    	$(cell).text(channel['status']);
    	}
	},
}