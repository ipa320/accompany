function dataHelper() {
}

dataHelper.prototype = {
	getLinks : function() {
		var links = null;
		$.ajax({
			url : 'js/links',
			dataType : 'json',
			async : false,
			success : function(data, textStatus, jqXHR) {
				links = data
			},
		});

		return links;
	},
}

function uiHelper() {
}

uiHelper.prototype = {
	load : function(root) {
		var links = new dataHelper().getLinks()

		for (index in links) {
			var link = links[index];
			var newLnk = $('<a></a>')
			newLnk.prop('href', link['path']);
			newLnk.text(link['title']);
			$(root).append(newLnk);
		}
	},
}
