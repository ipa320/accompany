'use strict';

/* App Module */

var historyApp = angular.module('historyApp', [
  'ngRoute',
  'ngTouch',
  'historyAppController',
  'ui.bootstrap'
]);


historyApp.config(['$routeProvider', '$locationProvider',
  function($routeProvider, $locationProvider) {
    $routeProvider.
      when('/history', {
        templateUrl: 'angular/partials/event-list.html',
        controller: 'EpisodesListCtrl'
      }).
      when('/history/:eventId/images/:imageUrl', {
        templateUrl: 'angular/partials/modal.html',
        controller: 'ImagesCtrl'
      }).
      when('/history/:eventId', {
        templateUrl: 'angular/partials/event-detail.html',
        controller: 'EpisodesDetailCtrl'
      }).
      otherwise({
        redirectTo: '/history'
      });
  }]);
