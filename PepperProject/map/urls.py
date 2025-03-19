# myapp/urls.py
from django.urls import path
from . import views

urlpatterns = [
    path('get-map/<int:map_id>/', views.get_map, name='get-map'),  # Fetch matrix
    path('save-map/', views.save_map, name='save-map'),  # Save matrix
    path('edit_map/', views.edit_map, name='edit_map'),  # Create new map
    path('edit_map/<str:map_name>/<int:map_id>/', views.edit_map, name='edit_map'),  # Edit existing map
    path('user-maps/', views.user_maps, name='user_maps'),  # List user maps
    path('delete-map/<int:map_id>/', views.delete_map, name='delete_map'),  # Delete map
    path('set-current-map/<int:map_id>/', views.set_current_map, name='set_current_map'),  # Set current map
]