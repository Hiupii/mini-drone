// Xử lý giao diện và các sự kiện chung

// Hiển thị thông tin chi tiết của ảnh
const viewDetailsButton = document.getElementById('view-details');
const detailsDiv = document.getElementById('details');

viewDetailsButton.addEventListener('click', function () {
  detailsDiv.style.display = detailsDiv.style.display === 'none' ? 'block' : 'none';
});
