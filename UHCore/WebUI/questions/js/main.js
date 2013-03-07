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
			url : 'data/current',
			dataType : 'json',
			async : false,
			timeout: 3000,
			success : function(data, textStatus, jqXHR) {
				responses = data
			},
		});

		return responses;
	},

	pollResponses : function(callback, root, lastId) {
		var self = this;
		var data = self.getResponses();
		var id = callback(root, lastId, data);
		setTimeout(function() {
			self.pollResponses(callback, root, id);
		}, 2000);
	},
}

function uiHelper() {
}

uiHelper.prototype = {
	load : function() {
		var dao = new dataHelper();
		dao.pollResponses(this.fill.bind(this), '#questions', -1);
	},

	fill : function(root, lastId, data) {
		if (lastId != data['query']) {
			$(root).empty();
			//var data = {'query': 'none'/id, 'reqponses': ({'guiResponseId':1, 'message':'do this', 'size': 's/m/l'})};
			if (data['query'] != 'none') {
				$('.question')
				for (index in data['responses']) {
					var response = data['responses'][index]
					var newQuery = $('<div></div>')
					var self = this;
					$(newQuery).click(function() {
						self.answerQuestion(this, data['query']);
					});
					newQuery.attr('responseId', response['guiResponseId']);
					newQuery.attr('size', response['size']);
					class_ = 'question'
					if (index % 2 == 0) {
						class_ += ' even'
					} else {
						class_ += ' odd'
					}

					newQuery.prop('class', class_);
					newQuery.text(response['message']);
					$(root).append(newQuery);
				}
			} else {
				var newQuery = $('<div></div>')
				newQuery.prop('class', 'center big');
				newQuery.text("Thank You.  I don't have any more questions right now.");
				$(root).append(newQuery);
			}
		}

		return data['query']
	},

	answerQuestion : function(element, question) {
		var id = $(element).attr('responseId');
		var url = 'data/' + question
		var result = {};
		var obj = {
			'response' : id
		};
		$.ajax({
			url : url,
			data : JSON.stringify(obj),
			async : false,
			contentType : 'application/json',
			error : function(jqXHR, status, error) {
				result = {
					status : status,
					error : jqXHR.responseText
				};
			},
			success : function(data, status, jqXHR) {
				result = {
					status : status,
					data : data
				};
			},
			type : 'POST'
		});

		if (result.status == 'success') {
			location.reload(true);
		} else {
			//Handle errors?
			alert(result.status);
		}
	},
}
