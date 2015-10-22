'use strict';

/* App Module */

var historyApp = angular.module('historyApp', [
  'ngRoute',
  'historyControllers'
]);
 
historyApp.config(['$routeProvider',
  function($routeProvider) {
    $routeProvider.
      when('/history', {
        templateUrl: 'partials/events-list.html',
        controller: 'EpisodesListCtrl'
      }).
      otherwise({
        redirectTo: ''
      });
  }]);
