from django.shortcuts import render
from django.shortcuts import render_to_response
from rest_framework.views import APIView
from rest_framework.response import Response
from rest_framework import status
from drone_app.serializers import LocationSerializer
from drone_app.models import Location

# Create your views here.



class LocationView(APIView):

	def get(self,request,format=None):
		location = Location.objects.latest('id')
		serializer = LocationSerializer(location,many=False)
		return Response(serializer.data)




