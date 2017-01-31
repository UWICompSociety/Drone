from __future__ import unicode_literals

from django.db import models
from django.utils import timezone

# Create your models here.



class Location(models.Model):
	name = models.CharField(max_length=255,default='test')
	lat  = models.CharField(max_length=255)
	lon  = models.CharField(max_length=255)
	time = models.DateTimeField('auto_now_add=true',default=timezone.now(),editable=False)

	def __str__(self):
		return self.name

