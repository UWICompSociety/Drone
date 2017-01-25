# -*- coding: utf-8 -*-
# Generated by Django 1.9.8 on 2017-01-10 20:52
from __future__ import unicode_literals

import datetime
from django.db import migrations, models
from django.utils.timezone import utc


class Migration(migrations.Migration):

    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='Location',
            fields=[
                ('id', models.AutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('lat', models.CharField(max_length=255)),
                ('lon', models.CharField(max_length=255)),
                ('time', models.DateTimeField(default=datetime.datetime(2017, 1, 10, 20, 52, 55, 554762, tzinfo=utc), editable=False, verbose_name='auto_now_add=true')),
            ],
        ),
    ]