'use strict';

/* Controllers */

//var historyApp = angular.module('historyApp', [ 'ui.bootstrap', '$strap.directives']);

historyApp.controller('EpisodesListCtrl', function EpisodesListCtrl($scope, $http) {
  $http.get('data/events/').success(function(data) {
    $scope.episodes = data;
    $scope.tagsValue = '0';
    $scope.ruleName = [];
    $scope.slides = [];
    $scope.locationName = [];
    $scope.scenarioName = [];
  });

  $scope.tagsItems = [{ label: "memorable", value: "important" },{ label:"interesting", value: "interesting" },{ label:"unclear", value: "question" },{ label:"none", value: "0" }];
  $scope.orderProp = 'time.narrative';  
  $scope.orderEpisodes = '-endTime'; 

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

  $scope.loadScenarioName = function (e) {
    //$scope.ruleName.push(e);
    //add if it's not repeated
    if ($scope.scenarioName.indexOf(e)==-1) $scope.scenarioName.push(e);
  }

  $scope.loadCarousel = function (txt, img){
    $scope.slides.push({text: txt, image: img});
  }

  $scope.loadRuleName = function (e) {
    //$scope.ruleName.push(e);
    //add if it's not repeated
    if ($scope.ruleName.indexOf(e)==-1) $scope.ruleName.push(e);
  }

  $scope.clear_eventquery = function() {
    $scope.eventquery="";
  }

  $scope.clear_locationquery = function() {
    $scope.locationquery="";
  }

  $scope.viaService = function() {
    
  }

});


historyApp.controller('ModalCtrl', function ($scope, $modal){
$scope.viaService = function() {
    // create the modal
    var modal = $modal({
      template: 'angular/js/app/modal.html',
      show: true,
      scope: $scope,
      persist: true,
      backdrop: 'static'
    });
$scope.parentController = function(dismiss) {
    console.warn(arguments);
    // do something
    dismiss();
    }
  }
});

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
