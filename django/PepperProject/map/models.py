# myapp/models.py
from django.db import models
from django.contrib.auth.models import User

class Map(models.Model):
    id = models.AutoField(primary_key=True)  # Explicitly define the id field
    user = models.ForeignKey(User, on_delete=models.CASCADE, related_name='maps')
    name = models.CharField(max_length=255)
    matrix = models.JSONField()  # Stores the map matrix as a JSON array
    is_default = models.BooleanField(default=False)  # Indicates if this is the default map
    is_current = models.BooleanField(default=False)  # Indicates if this is the current map
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    def __str__(self):
        return f"{self.name} (User: {self.user.username})"