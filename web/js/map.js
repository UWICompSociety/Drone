"use strict";

function initMap() {
    var map;
    var dropoff;

    map = new google.maps.Map(document.getElementById('map'), {
      center: {lat: 18.0050379, lng: -76.7488265},
      zoom: 17
    });

    map.addListener('click', function(e) {
        if (dropoff == undefined) {
            dropoff = new google.maps.Marker({
                position: e.latLng,
                map: map,
                title: 'Dropoff'
            });
        } else {
            dropoff.setPosition(e.latLng);
        }
    });

    // Try HTML5 geolocation.
    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition(function(position) {
        var pos = {
            lat: position.coords.latitude,
            lng: position.coords.longitude
        };

        map.setCenter(pos);
        map.setZoom(19);
      }, function() { /* TODO: handle location error */ });
    }
}
