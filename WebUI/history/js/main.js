function dataHelper() {
}

dataHelper.prototype = {
	getEvents : function(key) {
		var eventData = null;
		$.ajax({
			//Key not currently implemented
			url : 'data/events/' + key,
			dataType : 'json',
			async : false,
			success : function(data, textStatus, jqXHR) {
				eventData = data['Episodes']
			},
		});

		return eventData;
	},
}

function uiHelper() {
}

uiHelper.prototype = {
	load : function(episodes, root) {
		//var episodes = ['history']['EpisodicMemory']['AutobiographicalMemory']['Episodes'];
		var list = $('<ul></ul>');
		$(list).prop('id', 'Overview');
		$(list).attr('selected', 'true');
		$(list).prop('title', 'Overview');
		$(root).append(list);

		for (index in episodes) {
			var episode = episodes[index];
			var newEp = $('<li></li>')
			newEp.prop('class', 'group');
			newEp.text('Interactive Session ' + (index + 1));
			list.append(newEp);

			var events = episode['Events'];
			for (eventIndex in events) {
				if (eventIndex == 0 || events[eventIndex]['status'] == 'activate') {
					var event = events[eventIndex];
					var newEv = $('<li></li>');
					var newLnk = $('<a></a>');
					var timestamp = event['time']['narrative'];
					newLnk.prop('href', '#' + timestamp);
					newLnk.text(event['time']['real'] + ': ' + event['name']);
					newEv.append(newLnk);

					var details = $('<ul></ul>');
					$(details).prop('id', timestamp);
					$(details).prop('title', 'Event Details');
					this.appendDetail(details, 'Time', event['time'], 'real');
					this.appendDetail(details, 'Location', event, 'location');
					this.appendDetail(details, 'Action Name', event, 'name');
					this.appendDetail(details, 'Target', event, 'target');

					if (event['imageUrl'] != undefined) {
						details.append(this.createListItem('group', 'Image'));
						var img = $('<img></img>');
						var self = this;
						$(img).load(function() {
							self.toggleSize(this, true);
						});
						$(img).click(function() {
							self.toggleSize(this);
						});
						$(img).prop('src', event['imageUrl']);
						details.append(this.createListItem(null, img));
					}

					if (event['parameters'] != undefined && event['parameters'].length > 0) {
						details.append(this.createListItem('group', 'Other Information'));
						for (index in event['parameters']) {
							details.append(this.createListItem(null, event['parameters'][index]['name'] + ': ' + event['parameters'][index]['value']));
						}
					}

					if (event['sensorMapUrl'] != undefined) {
						details.append(this.createListItem('group', 'Sensor Map'));
						var img = $('<img></img>');
						$(img).prop('src', event['sensorMapUrl']);
						var self = this;
						$(img).load(function() {
							self.toggleSize(this, true);
						});
						$(img).click(function() {
							self.toggleSize(this);
						});
						details.append(this.createListItem(null, img));
					}

					if (event['sensors'] != undefined && event['sensors'].length > 0) {
						details.append(this.createListItem('group', 'Sensor Snapshot'));
						for (sensorIndex in event['sensors']) {
							details.append(this.createListItem(null, event['sensors'][sensorIndex]['location'] + ' ' + event['sensors'][sensorIndex]['name'] + ': ' + event['sensors'][sensorIndex]['value']));
						}
					}

					$(root).append(details);
					$(list).append(newEv);
				}
			}
		}
	},

	appendDetail : function(list, title, data, propName) {
		var dataNode = null;
		if (data != null && data != undefined) {
			if (data[propName] != undefined) {
				var dataNode = this.createListItem(null, data[propName]);
			}
		}

		if (dataNode != null) {
			var titleNode = this.createListItem('group', title);
			list.append(titleNode);
			list.append(dataNode);
		}
	},

	createListItem : function(cls, content) {
		var node = $('<li></li>');
		if (cls != null && cls != undefined) {
			node.prop('class', cls);
		}

		if (content != null && content != undefined) {
			node.append(content);
		}

		return node;
	},

	toggleSize : function(image, forceSmall) {
		if (forceSmall == undefined) {
			forceSmall = false;
		}

		var thumbWidth = 160
		var width = image.width;
		var height = image.height;
		var ratio = width / height

		if (width > thumbWidth || $(image).prop('width') == 0 || forceSmall) {
			$(image).prop('width', thumbWidth);
			$(image).prop('height', thumbWidth / ratio);
		} else {
			$(image).prop('width', $('body').width() - 20);
			$(image).prop('height', ($('body').width() - 20) / ratio);
		}
	},

	resize : function(node, height, width) {
		$(node).prop('height', height);
		$(node).prop('width', width);
	},
}
