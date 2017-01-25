from rest_framework import serializers
from drone_app.models import Location




class LocationSerializer(serializers.ModelSerializer):
	class Meta:
		model = Location
		fields = ('lat','lon','time')