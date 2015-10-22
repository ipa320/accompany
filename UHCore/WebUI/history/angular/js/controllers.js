'use strict';

/* Controllers */

var historyAppController = angular.module('historyAppController', ['ui.bootstrap']);

historyAppController.controller('EpisodesListCtrl', function EpisodesListCtrl($scope, $http, $location) {
  $http.get('data/events/').success(function(data) {
    $scope.episodes = data;
  });

  $scope.orderEpisodes = 'endTime'; 

  $scope.go = function ( path ) {
    $location.path( "/history/"+path );
  };

  $scope.clearScenarioQuery = function() {
    $scope.scenarioquery="";
  }

});

historyAppController.controller('ImagesCtrl', function ImagesCtrl($scope, $http, $location,$routeParams) {
  $http.get('data/events/').success(function(data) {
    $scope.img = data;
  });

  $scope.img2Show = $routeParams.imageUrl;
  $scope.eventId2Show = $routeParams.eventId;

  $scope.goback = function ( path  ) {
    $location.path( "/history/"+path);
  };

});


historyAppController.controller('EpisodesDetailCtrl', function EpisodesDetailCtrl($scope, $routeParams, $http, $location){
  $http.get('data/events/').success(function(data) {
    $scope.episodes = data;
    $scope.tagsValue = '0';
    $scope.ruleName = [];
    $scope.locationName = [];
  });

  $scope.eventId2Show = $routeParams.eventId;

  $scope.tagsItems = [{ label: "memorable", value: "important" },{ label:"interesting", value: "interesting" },{ label:"unclear", value: "question" },{ label:"none", value: "0" }];
  $scope.orderProp = 'time.narrative';

  $scope.go2 = function ( path  ) {
    $location.path( "/history/" +$scope.eventId2Show+"/"+path);
  };
  //$scope.ruleName = 'name';

  $scope.goback = function () {
    $location.path( "/history");
  };

  $scope.setRadioValue=function(radioVal){
    $scope.tagsValue=radioVal;
    //$scope.ruleName.push(even);
  }

  $scope.setTags=function(tags,event){//this function should set the tags on the database
    $scope.dao= new dataHelper();
    $scope.dao.setTags(event.id, tags);
    event.tags[0]=tags;
    //$scope.ruleName.push(event.name);
  }

  $scope.loadLocationName = function (e) {
    //$scope.ruleName.push(e);
    //add if it's not repeated
    if ($scope.locationName.indexOf(e)==-1) $scope.locationName.push(e);
  }

  $scope.loadRuleName = function (e) {
    if ($scope.ruleName.indexOf(e)==-1) $scope.ruleName.push(e);
  }

  $scope.clearEventQuery = function() {
    $scope.eventquery="";
  }

  $scope.clearLocationQuery = function() {
    $scope.locationquery="";
  }


 });

// historyAppController.controller('EpisodesDetailCtrl', function ($scope, $routeParams, $http){
//   $http.get('json/' + $routeParams.eventId + '.json').success(function(data) {
//    $scope.episodes = data;
//   });
//  // $scope.eventId = $routeParams.eventId;
//  });

//historyApp.controller('TagsCtrl', function TagsCtrl($scope,$html){
//  $http.post('data/tags/').success(function(data){
//    $scope.tag=data;
//  });
//});
//historyControllers.controller('EpisodesDetailCtrl', ['$scope', '$routeParams',
//  function($scope, $routeParams) {
//    $scope.phoneId = $routeParams.phoneId;
//  }]);

function dataHelper() {
}

dataHelper.prototype = {
  setTags : function(historyId, tags) {
    var url = 'data/tags'
    var result = {};
    var obj = {
      'historyId' : historyId,
      'tags' : tags
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
  }
}
