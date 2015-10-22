'use strict';

/* Controllers */

//var historyApp = angular.module('historyApp', []);

var historyControllers = angular.module('historyControllers', []);


historyControllers.controller('EpisodesListCtrl', ['$scope', '$http',
  function EpisodesListCtrl($scope, $http) {
    $http.get('data/events/').success(function(data) {
      $scope.episodes = data;
    });
  }]);

/*historyControllers.controller('EpisodesListCtrl', ['$scope', '$http', 
	function EpisodesListCtrl($scope, $http) {
	  $http.get('data/events/').success(function(data) {
		$scope.episodes = data;
	  });
	}]);
  
historyApp.controller('EpisodesListCtrl', function EpisodesListCtrl($scope, $http) {
  $http.get('data/events/').success(function(data) {
    $scope.episodes = data;
  });
});*/

//historyControllers.controller('EpisodesDetailCtrl', ['$scope', '$routeParams',
//  function($scope, $routeParams) {
//    $scope.phoneId = $routeParams.phoneId;
//  }]);
